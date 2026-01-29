#pragma once

#include "Eigen/Eigen"
#include "units/units.hpp"
#include "sensor.h"

#include <random>
#include <algorithm>
#include <cmath>
#include <vector>
#include <array>
#include <functional>
#include <iostream>

#include "config.h"

struct Particle {
    float x;
    float y;
    float theta;
    float weight = 1.0f;
};

template<size_t L>
class ParticleFilter {
    // Ensure particle count is reasonable
    static_assert(L <= 500, "Too many particles, reduce to 500 or less.");

private:
    std::array<Particle, L> particles;
    std::array<Particle, L> oldParticles; // For resampling

    Eigen::Vector3f prediction{}; // x, y, theta

    std::vector<Sensor*> sensors;

    QLength distanceSinceUpdate = 0.0;
    QTime lastUpdateTime = 0.0;

    QLength maxDistanceSinceUpdate = 2_in;
    QTime maxUpdateInterval = 2.0_s;

    std::ranlux24_base de;
    std::uniform_real_distribution<> fieldDist{-1.78308, 1.78308}; // Meters
    std::uniform_real_distribution<> angleDist{-M_PI, M_PI};

public:
    ParticleFilter() {
        // Random seed
        std::random_device rd;
        de.seed(rd());

        for (auto& p : particles) {
            p.x = 0.0f;
            p.y = 0.0f;
            p.theta = 0.0f;
            p.weight = 1.0f / L;
        }
    }

    Eigen::Vector3f getPrediction() {
        return prediction;
    }

    // Helper to get raw particles if needed for vis/debug
    const std::array<Particle, L>& getParticles() const {
        return particles;
    }

    Eigen::Vector3f getParticle(size_t i) {
        // Return mostly for interface compatibility
        return {particles[i].x, particles[i].y, particles[i].theta};
    }

    float weightParticle(const Particle &p) {
        float totalWeight = 1.0f;

        // Construct Vector3f for sensor compatibility for now
        // Assuming sensor->p takes Vector3f(x, y, theta)
        Eigen::Vector3f pose(p.x, p.y, p.theta);

        for (const auto sensor : sensors) {
            if (auto w = sensor->p(pose); w.has_value() && std::isfinite(w.value())) {
                totalWeight *= static_cast<float>(w.value());
            }
        }
        return totalWeight;
    }

    void updateSensors() {
        for (auto &&sensor : this->sensors) {
            sensor->update();
        }
    }

    // Update takes a function that modifies a particle in-place (Motion Model)
    // The RNG is passed so the motion model can use the same generator or we can manage it there.
    // Ideally, the motion model encapsulates the noise generation.
    void update(const std::function<void(Particle&)>& motionModel) {
        
        auto start = pros::micros();

        // 1. Motion Update
        float avgMotion = 0.0f; 
        for (auto& p : particles) {
            float oldX = p.x;
            float oldY = p.y;
            
            motionModel(p); // Apply motion (with noise)
            
            // Normalize Theta
            while (p.theta > M_PI) p.theta -= 2 * M_PI;
            while (p.theta <= -M_PI) p.theta += 2 * M_PI;
            
            float dist = std::sqrt(std::pow(p.x - oldX, 2) + std::pow(p.y - oldY, 2));
            avgMotion += dist;
        }
        avgMotion /= L;
        distanceSinceUpdate += avgMotion * meter; // Assuming implicit conversion or adding double

        // Check if we should resample (Update Step)
        if (distanceSinceUpdate < maxDistanceSinceUpdate && 
            maxUpdateInterval > pros::millis() * millisecond - lastUpdateTime) {
            
            // Just update prediction expectation without resampling
            calculatePrediction();
            return;
        }

        // 2. Measurement Update
        updateSensors();

        double totalWeight = 0.0;
        double maxWeight = 0.0;

        for (size_t i = 0; i < L; i++) {
            // Handle field bounds (kidnapped robot / robustness)
            if (outOfField(particles[i])) {
               // Re-init lost particles? Or just let them die?
               // Current logic: random re-init (injection)
               particles[i].x = fieldDist(de);
               particles[i].y = fieldDist(de);
               particles[i].theta = angleDist(de);
               particles[i].weight = 0.0f; // Will get weighted next
            }

            particles[i].weight = weightParticle(particles[i]);
            totalWeight += particles[i].weight;
            if (particles[i].weight > maxWeight) maxWeight = particles[i].weight;
        }

        if (totalWeight <= 1e-6) {
             // Total weight collapse: Rescue with uniform distribution?
             // Or print warning and skip resampling
             // std::cout << "Warning: Weight collapse" << std::endl;
             return; 
        }

        // 3. Resampling (Low Variance Sampler)
        // Normalized weights are implied in the logic usually, but here we use the specific Low Variance Algo
        
        // Copy current to old for source
        oldParticles = particles;
        
        double avgWeight = totalWeight / static_cast<double>(L);
        // Safety check
        if (avgWeight <= 0.0) avgWeight = 1e-9;

        std::uniform_real_distribution<double> dist(0.0, totalWeight / L); // r ~ U(0, 1/M) * sum(w) is equiv to U(0, avgWeight) ?? 
        // Standard Low Variance: 
        // r = uniform(0, M^-1)
        // c = w[0]
        // i = 0
        // for m = 1 to M:
        //    U = r + (m-1) * M^-1
        //    while U > c:
        //       i++
        //       c += w[i]
        //    add p[i] to new set
        
        // The previous implementation used:
        // const double avgWeight = totalWeight / L;
        // std::uniform_real_distribution distribution(0.0, avgWeight);
        // double randWeight = distribution(de);
        // ... loop ...
        // weight = i * avgWeight + randWeight;
        // while cumulative < weight ...
        
        // This is correct for Low Variance Sampling (Stochastic Universal Sampling).
        
        std::uniform_real_distribution distribution(0.0, avgWeight);
        const double initialRand = distribution(de);
        
        size_t sourceIdx = 0;
        double cumulativeWeight = oldParticles[0].weight;
        
        for (size_t i = 0; i < L; i++) {
            double targetSum = static_cast<double>(i) * avgWeight + initialRand;
            
            while (cumulativeWeight < targetSum && sourceIdx < L - 1) {
                sourceIdx++;
                cumulativeWeight += oldParticles[sourceIdx].weight;
            }
            
            // Copy state
            particles[i] = oldParticles[sourceIdx];
            particles[i].weight = 1.0f; // Reset weight after resampling
        }

        calculatePrediction();

        lastUpdateTime = pros::millis() * millisecond;
        distanceSinceUpdate = 0.0;
    }

    void calculatePrediction() {
        float xSum = 0.0, ySum = 0.0;
        float sinSum = 0.0, cosSum = 0.0;

        for (size_t i = 0; i < L; i++) {
            xSum += particles[i].x;
            ySum += particles[i].y;
            sinSum += std::sin(particles[i].theta);
            cosSum += std::cos(particles[i].theta);
        }

        float meanTheta = std::atan2(sinSum, cosSum);
        prediction = Eigen::Vector3f(xSum / L, ySum / L, meanTheta);
    }

    void initNormal(const Eigen::Vector3f& mean, const Eigen::Matrix3f& covariance) {
        // Simple distinct normal dists for x, y, theta for now
        // Assuming covariance is diagonal or we just use diag elements
        std::normal_distribution<float> xDist(mean.x(), std::sqrt(covariance(0,0)));
        std::normal_distribution<float> yDist(mean.y(), std::sqrt(covariance(1,1)));
        std::normal_distribution<float> tDist(mean.z(), std::sqrt(covariance(2,2)));

        for (auto & p : this->particles) {
            p.x = xDist(de);
            p.y = yDist(de);
            p.theta = tDist(de);
            
             // Normalize Theta
            while (p.theta > M_PI) p.theta -= 2 * M_PI;
            while (p.theta <= -M_PI) p.theta += 2 * M_PI;
        }
        calculatePrediction();
    }

    static bool outOfField(const Particle& p) {
        // Hardcoded field limits from config/distance.h
        return p.x > 1.78308 || p.x < -1.78308 || p.y < -1.78308 || p.y > 1.78308;
    }

    void initUniform(const QLength minX, const QLength minY, const QLength maxX, const QLength maxY) {
        std::uniform_real_distribution<float> xDistribution(minX.getValue(), maxX.getValue());
        std::uniform_real_distribution<float> yDistribution(minY.getValue(), maxY.getValue());
        std::uniform_real_distribution<float> tDistribution(-M_PI, M_PI);

        for (auto & p : this->particles) {
            p.x = xDistribution(de);
            p.y = yDistribution(de);
            p.theta = tDistribution(de);
            p.weight = 1.0f / L;
        }
        calculatePrediction();
    }

    void addSensor(Sensor* sensor) {
        this->sensors.emplace_back(sensor);
    }
};
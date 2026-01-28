// #include "main.cpp"

// Intake::Intake(pros::Motor bottom_intake, pros::Motor top_intake, pros::adi::DigitalOut piston1) : bottom_intake(bottom_intake), top_intake(top_intake), piston1(piston1){}

// extern void set_score_piston(bool state);


// void Intake::telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake){
//     if(outtake){
//         top_intake.move_velocity(-600);
//         bottom_intake.move_velocity(-600);
//     }
//     else if(intake){
//         bottom_intake.move_velocity(600);
//     }
//     else if(scoreTop){
//         bottom_intake.move_velocity(600);
//         top_intake.move_velocity(600);
//     }
//     else if(scoreMid){
//         bottom_intake.move_velocity(600);
//         top_intake.move_velocity(600);
//         set_score_piston(state, piston1);
//     }
// }
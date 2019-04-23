#include <Kin/kin.h>
#include <RosCom/baxter.h>
#include <Operate/robotOperation.h>

void minimal_use(){
  rai::KinematicWorld K;
  K.addFile("../../rai-robotModels/baxter/baxter.g");
  arr q0 = K.getJointState();

  BaxterInterface B(true);
  B.send_q(q0);

  for(uint i=0;i<10;i++){
    rai::wait(.1);
    cout <<B.get_q() <<endl;
    cout <<B.get_qdot() <<endl;
    cout <<B.get_u() <<endl;
  }
  K.watch(true);

  arr q = q0;
  q = 0.;
  K.setJointState(q);
  B.send_q(q);
  K.watch(true);
}

void spline_use(){
  rai::KinematicWorld K;
  K.addFile("../../rai-robotModels/baxter/baxter.g");
  K.addObject("object", rai::ST_capsule, {.2, .05}, {0., 1., 0.}, -1., 0, {.8, .0, 1.});
  //K.addObject("object", rai::ST_capsule, {.4, .1}, {1., 1., 0.}, -1., 0, {.8, .0, 1.});
  //K.addObject(name="ball", shape=ST.sphere, size=[.1], pos=[.8,.8,1.5], color=[1,1,0])
  // {shape}, {color}, -1., 0, {x y z coord}
  arr q_home = K.getJointState();

  arr q_zero = 0.*q_home;

  RobotOperation B(K);
  cout <<"joint positions: " <<B.getJointPositions() <<endl;
  cout <<"joint names: " <<B.getJointNames() <<endl;
  B.move({q_zero,q_home}, {5.,10.});
  B.move({q_zero}, {15.}); //appends
  B.wait();
  rai::wait();

  q_home(-1) = .1; //last joint set to .1: left gripper opens 10cm (or 20cm?)
  B.move({q_home}, {4.});
  B.wait();

  rai::wait();
}

//ex01:
void grasp_use_old(){
  rai::KinematicWorld K;
  K.addFile("../../rai-robotModels/baxter/baxter.g");
  K.addObject("object", rai::ST_capsule, {.2, .05}, {1., 1., 0.}, -1., 0, {.8, .0, 1.});
  //K.addObject("object", rai::ST_capsule, {.4, .1}, {1., 1., 0.}, -1., 0, {.8, .0, 1.});
  //K.addObject(name="ball", shape=ST.sphere, size=[.1], pos=[.8,.8,1.5], color=[1,1,0])
  // {shape}, {color}, -1., 0, {x y z coord}
  arr q_home = K.getJointState(); //array of 17 init pos, closed

  //arr q_zero = 0.*q_home; //max open hands

  //K.getJointStateDimension();
  arr y_target, y_vec_start, y_posit_start, J, y_linear;
  //arr y_vec_end = {-1., 0., 0.}; // RGB

  double T = 100;
  arr Phi, PhiJ;
  arr y_left, J_left;

  // left_wrist is a joint
  // left_lower_forearm

  //use Z dim of obj
  K.evalFeature(y_vec_start, J, FS_vectorZ, {"left_lower_forearm"});
  //K.evalFeature(y_vec_start, J, FS_scalarProductZZ, {"baxterL", "object"});
  //FS_scalarProductZZ, {"baxterL", "object"});
  
  //position of obj
  K.evalFeature(y_posit_start, J, FS_position, {"baxterL"});
  //y_posit_start += y_vec_start*-0.4;

  arr q_task = K.getJointState();

  RobotOperation B(K);
  cout <<"joint positions: " <<B.getJointPositions() <<endl;
  cout <<"joint names: " <<B.getJointNames() <<endl;
  //B.move({q_zero,q_home}, {5.,10.}); //
  //B.move({q_zero}, {15.}); //appends
  //B.wait();
  //rai::wait();

  arr q, W;
  uint n = K.getJointStateDimension();
  K.getJointState(q);
  double w = rai::getParameter("w", 1e-4);
  W.setDiag(w, n);  //W is equal the Id_n matrix times scalar w

  arr y_vec, J_vec;
  y_target = {.8, .0, 1.};

  for(uint t=0; t<100; t++){
        
    // vector
    //K.evalFeature(y_vec, J_vec, FS_vectorZ, {"left_lower_forearm"}); // old
    // linear profile iterative  ? use with scalar prod
    //y_linear = y_vec_start + (t/T)*(y_vec_end - y_vec_start); // 
    //Phi.append( (y_linear - y_vec)/10. );

    // ex01 task2: align gripper
    // scalar prod zero btw R and B of hand and Z of object
    K.evalFeature(y_vec, J_vec, FS_scalarProductXZ, {"baxterL", "object"});
    Phi.append( y_vec/10. );
    PhiJ.append( J_vec/10. );
    

    K.evalFeature(y_vec, J_vec, FS_scalarProductZZ, {"baxterL", "object"});
    Phi.append( y_vec/10. );
    PhiJ.append( J_vec/10. );
    
    // position
    K.evalFeature(y_left, J_left, FS_position, {"baxterL"}); //left ..
    y_linear = y_posit_start + (t/T)*(y_target - y_posit_start); // 
    Phi.append((y_linear - y_left));
    PhiJ.append(J_left);


    //compute joint updates //ex01 task1: move arm towards obj
    //y_linear = y + (t/T)*(y_target - y);
    //q += 0.1*inverse(~J*J + W)*~J*(y_linear - y); 
    q += inverse(~PhiJ*PhiJ + W)*~PhiJ*Phi;
    //NOTATION: ~J is the transpose of J
    //TODO use linear motion interpolation, not just (y_target - y) !?
    
    //sets joint angles AND computes all frames AND updates display
    K.setJointState(q);

    //optional: pause and watch OpenGL
    //K.watch(true);
    B.move({q}, {4./T});
    B.wait();
  }

  //K.setJointState(q);
  //B.move();

  //Z dim of obj
  //K.evalFeature(y, J, FS_vectorZ, {"left_lower_forearm"});


  //q_home(-1) same as q_home(16)
  //q_home(-1) = .1; //last joint set to .1: left gripper opens 10cm (or 20cm?)
  //B.move({q_home}, {4.});
  //q_home(-1) = 0.; //left gripper close?
  //B.wait();

  q(-1) = .05; //last joint set to .1: left gripper opens 10cm (or 20cm?)
  B.move({q}, {4.});
  q(-1) = 0.; //left gripper close?
  B.move({q}, {4.});
  B.wait();

  rai::wait();
}



void grasp_use(){
  rai::KinematicWorld K;
  K.addFile("../../rai-robotModels/baxter/baxter.g");
  K.addObject("object", rai::ST_capsule, {.2, .05}, {0., 1., 0.}, -1., 0, {.8, .0, 1.});
  arr q_home = K.getJointState(); //array of 17 init pos, closed

  //arr q_zero = 0.*q_home; //max open hands

  //K.getJointStateDimension();
  arr y_target, y_vec_start, y_posit_start, J, y_linear;
  //arr y_vec_end = {-1., 0., 0.}; // RGB

  double T = 100;
  arr Phi, PhiJ;
  arr y_left, J_left;

  // left_wrist is a joint
  // left_lower_forearm
  

  //position of obj
  K.evalFeature(y_posit_start, J, FS_position, {"baxterL"});

  arr q_task = K.getJointState();

  RobotOperation B(K);
  //cout <<"joint positions: " <<B.getJointPositions() <<endl;
  cout <<"joint names: " <<B.getJointNames() <<endl;
  //B.move({q_zero,q_home}, {5.,10.}); //
  //B.move({q_zero}, {15.}); //appends
  //B.wait();
  //rai::wait();

  arr q, W;
  uint n = K.getJointStateDimension();
  K.getJointState(q);
  double w = rai::getParameter("w", 1e-4);
  W.setDiag(w, n);  //W is equal the Id_n matrix times scalar w

  arr y_vec, J_vec;
  y_target = {.8, .0, 1.};
  double sum = .06, add = 0.;

  for(uint t=0; t<T; t++){
    Phi.clear(); 
    PhiJ.clear();
    
    // vector  // ex01 task2: align gripper
    // scalar prod zero btw R and B of hand and Z of object
    K.evalFeature(y_vec, J_vec, FS_scalarProductXZ, {"baxterL", "object"}); // <R,Z> = 0
    Phi.append( y_vec/T ); // /10.
    PhiJ.append( -J_vec/T );

    K.evalFeature(y_vec, J_vec, FS_scalarProductZZ, {"baxterL", "object"}); // <B,Z> = 0
    Phi.append( y_vec/T );
    PhiJ.append( -J_vec/T );
    
    // position
    /*
    K.evalFeature(y_left, J_left, FS_position, {"baxterL"}); //left ..
    y_linear = y_posit_start + (t/T)*(y_target - y_posit_start); // 
    Phi.append((y_linear - y_left));
    PhiJ.append(J_left);
    */
    K.evalFeature(y_left, J_left, FS_positionDiff, {"baxterL", "object"});
    Phi.append( y_left/T );
    PhiJ.append( -J_left/T );
    // similar penalization very important ?
    // why -Jacobian ??

    //compute joint updates //ex01 task1: move arm towards obj
    //y_linear = y + (t/T)*(y_target - y);
    //q += 0.1*inverse(~J*J + W)*~J*(y_linear - y); 
    q += inverse(~PhiJ*PhiJ + W)*~PhiJ*Phi; // q∗ = q0 + J#*(y∗−y0) 2.30
    //NOTATION: ~J is the transpose of J
    //TODO use linear motion interpolation, not just (y_target - y) !?
    
    //sets joint angles AND computes all frames AND updates display
    K.setJointState(q);

    //optional: pause and watch OpenGL
    //K.watch(true);

    // open grsper at the same time while moving a hand
    add += .06/(t+10);
    if (sum > add)
      q(-1) += .06/(t+10);

    B.move({q}, {4./T});
    B.wait();
  }

  //K.setJointState(q);
  //B.move();

  // do it at the same time while moving a hand
  //q(-1) = .06; //last joint set to .1: left gripper opens 10cm (or 20cm?)
  //B.move({q}, {4.});
  q(-1) = 0.; //left gripper close
  B.move({q}, {4.});
  B.wait();

  rai::wait();
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  //minimal_use();
  //spline_use();
  //grasp_use_old();
  grasp_use();

  return 0;
}

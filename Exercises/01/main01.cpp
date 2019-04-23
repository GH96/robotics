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
  K.addObject("object", rai::ST_capsule, {.2, .05}, {1., 1., 0.}, -1., 0, {.8, .0, 1.});
  arr q_home = K.getJointState();

  arr q_zero = 0.*q_home;

  RobotOperation B(K);
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



arr motionProfile(arr y_start,arr y_final,int i,double steps){
  arr diff = y_final - y_start;
  diff *= (i/steps);
  return y_start + diff;
}



void graspObject(){
  rai::KinematicWorld K;
  K.addFile("../../rai-robotModels/baxter/baxter.g");
  K.addObject("object", rai::ST_capsule, {.2, .05}, {1., 1., 0.}, -1., 0, {.8, .0, 1.});
  arr q_home = K.getJointState();
  FeatureSymbol fs_position = FS_position;
  arr q,W,y,J;
  q = q_home;
  uint n = K.getJointStateDimension();
  double w = rai::getParameter("w",1e-4);
  W.setDiag(w,n);

  arr q_zero = 0.*q_home;

  RobotOperation B(K);
  cout <<"joint names: " <<B.getJointNames() <<endl;
  //B.move({q_zero}, {10.});
  B.wait();
  K.setJointState(q_zero);
  rai::wait();

  arr y_final = {0.8,.0,1.};
  double steps = 10.;
  arr y_target;
  K.evalFeature(y, J, fs_position, {"left_lower_forearm"});
  arr y_start = y;
  
  for(int i=1; i<steps; i++){
    cout << "Eval Position " << i << endl;
    K.evalFeature(y, J, fs_position, {"left_lower_forearm"});
    
    y_target = motionProfile(y_start,y_final,i,steps);
    
    //cout << "y: " << y << endl;

    //cout << "y: " << y << " y_target: " << y_target << endl;
    q += inverse(~J*J + W)*~J*((y_target - y)); 
    //cout << "new joints: " << q << endl;
    B.move({q},{10./steps});
    B.wait();
    K.setJointState(q);
  }
  

  //q_home(-1) = .1; //last joint set to .1: left gripper opens 10cm (or 20cm?)
  //B.move({q_home}, {4.});
  //B.wait();


  rai::wait();
}


void graspObject1(){
  rai::KinematicWorld K;
  K.addFile("../../rai-robotModels/baxter/baxter.g");
  K.addObject("object", rai::ST_capsule, {.2, .05}, {1., 1., 0.}, -1., 0, {.8, .0, 1.});
  //K.addObject("object", rai::ST_ssBox, {.1,.3,.2, .001}, {1., 1., 0.}, -1., 0, {.8, .0, 1.},{.7,.7,.0,.0});
  arr q_home = K.getJointState();
  
  FeatureSymbol fs_position = FS_position;
  FeatureSymbol fs_vectorZ = FS_vectorZ;
  FeatureSymbol fs_qItself = FS_qItself;
  arr q,W,y,J,Phi,PhiJ;
  q = q_home;
  uint n = K.getJointStateDimension();
  double w = rai::getParameter("w",1e-4);
  W.setDiag(w,n);

  arr q_zero = 0.*q_home;

  RobotOperation B(K);
  cout << "joint names: " << B.getJointNames() << endl;
  cout << "Frame names: " << K.getFrameNames() << endl;
  cout << "Frame: " << K.getFrameByName("left_lower_forearm") << endl;
  rai::wait();

  

  arr y_final = {0.8,.0,1.};
  
  double steps = 100.;
  arr y_target, yvec,Jvec;

  // Orientation of arm
  arr vec_start, vec_target;
  arr vec_final = {-1.,0.,0.};
  K.evalFeature(vec_start, J, fs_vectorZ, {"left_lower_forearm"});

  // Position of arm 
  arr pos_start;
  K.evalFeature(pos_start, J, fs_position, {"left_lower_forearm"});
  pos_start += vec_start*-0.4;


  arr qcurrent, Jcurrent;




  
  for(int i=1; i<steps; i++){
    Phi.clear(); 
    PhiJ.clear();

    // Initial position of robot
    K.evalFeature(qcurrent,Jcurrent, fs_qItself, {});
    Phi.append((q_home - qcurrent)/100.);
    PhiJ.append(Jcurrent/100.);



    // Orientation of arm
    //K.evalFeature(yvec, Jvec,, {"left_lower_forearm"});
    K.evalFeature(yvec, Jvec, fs_vectorZ, {"left_lower_forearm"});
    vec_target = motionProfile(vec_start,vec_final,i,steps);
    Phi.append(vec_target - yvec);
    PhiJ.append(Jvec);

    
    // Position of arm
    K.evalFeature(y, J, fs_position, {"left_lower_forearm"});
    //K.kinematicsPos(y, J, K.getFrameByName("left_lower_forearm"),{0.,0.,0.});
    y_target = motionProfile(pos_start,y_final,i,steps);
    Phi.append(y_target - (y+yvec*-0.4));
    PhiJ.append(J);


    // Calculate Minimum...
    q += inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    K.setJointState(q);
    //B.move({q},{10./steps});
    //B.wait();
    
  }
  
  B.move({q},{10.});
  B.wait();
  //q_home(-1) = .1; //last joint set to .1: left gripper opens 10cm (or 20cm?)
  //B.move({q_home}, {4.});
  //B.wait();


  rai::wait();
}





void graspObject2(){
  rai::KinematicWorld K;
  K.addFile("../../rai-robotModels/baxter/baxter.g");
  K.addObject("object", rai::ST_capsule, {.2, .05}, {1., 1., 0.}, -1., 0, {.8, .0, 1.}/*,{0.7,.7,0,0}*/);
  arr q_home = K.getJointState();
  
  arr q,W,y,J,Phi,PhiJ;
  q = q_home;
  uint n = K.getJointStateDimension();
  double w = rai::getParameter("w",1e-4);
  W.setDiag(w,n);

  RobotOperation B(K);
  rai::wait();

  

  arr y_final = {0.8,.0,1.};
  
  double steps = 100.;
  arr y_target, yvec,Jvec;

  // Orientation of arm
  arr vec_start, vec_target;
  arr vec_final = {-1.,0.,0.};

  // Position of arm 
  arr pos_start;
  K.evalFeature(pos_start, J, FS_position, {"baxterL"});
  //pos_start += vec_start*-0.4;


  arr qcurrent, Jcurrent;




  
  for(int i=1; i<steps; i++){
    Phi.clear(); 
    PhiJ.clear();

    // Initial position of robot
    //K.evalFeature(qcurrent,Jcurrent, FS_qItself, {});
    //Phi.append((q_home - qcurrent)/1000.);
    //PhiJ.append(Jcurrent/1000.);



    // Orientation of arm
    K.evalFeature(yvec, Jvec, FS_scalarProductZZ, {"baxterL","object"});
    Phi.append(yvec/steps);
    PhiJ.append(-Jvec/steps);


    K.evalFeature(yvec, Jvec, FS_scalarProductXZ, {"baxterL","object"});
    Phi.append(yvec/steps);
    PhiJ.append(-Jvec/steps);

    
    // Position of arm
    K.evalFeature(y,J,FS_positionDiff,{"baxterL","object"});
    Phi.append(y/steps);
    PhiJ.append(-J/steps);

    //K.evalFeature(y, J, FS_position, {"baxterL"});
    //y_target = motionProfile(pos_start,y_final,i,steps);
    //Phi.append(y_target - (y));
    //Phi.append(y);
    //PhiJ.append(J);


    // Calculate Minimum...
    q += inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    K.setJointState(q);
    B.move({q},{10./steps});
    B.wait();
    
  }
  
  //B.move({q},{10.});
  //B.wait();
  //q_home(-1) = .1; //last joint set to .1: left gripper opens 10cm (or 20cm?)
  //B.move({q_home}, {4.});
  //B.wait();


  rai::wait();
}


















int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  //minimal_use();

  //spline_use();

  //graspObject();

  //graspObject1();


  graspObject2();

  return 0;
}

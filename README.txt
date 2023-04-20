CSCI 520, Assignment 3

Wanyu Zhang
USCID: 6773-4457-19

================
To graders:
This README.txt is used to describe the features and extra credits in a basic way.
For more details, please go to assignment report.

[Two Late Day policy Applied]

Run Command:
[RECOMMEND:]
In Visual Studio, simply change the "Command Arguments" on Property Page by adding two extra arguments after skin.config. The first one is DQS/LBS, the second one is TIK/PSEUDO.
DQS: Use dual-quaternion skinning method
LBS: Use linear blending skinning method
TIK: Use Tikhonov IK Solver
PSEUDO: Use Pseudoinverse IK Solver
For instance, 
"skin.config DQS TIK" means use DQS skinning and Tikhonov solver.
If only "skin.config" is given, it means running in default mode(LBS and Tikhonov)

[Directly use binaries might not succeed because of dependencies, if such happen, please use first method]
To directly run binaries built already, choose one of three resource folders(armadillo/dragon/hand), and run commands as below:
../output/IK.exe skin.config DQS TIK
../output/IK.exe skin.config DQS PSEUDO
../output/IK.exe skin.config LBS TIK
../output/IK.exe skin.config LBS PSEUDO
or 
../output/IK.exe skin.config
If only "skin.config" is given, it means running in default mode(LBS and Tikhonov)

[Note:] Under PSEUDO mode, you will see some artifacts if you move the handle too quickly, please refer to the report to see explainations.

Features:
- Implements Linear Blending Skinning method
- Implements Forward Kinematics
- Implements Inverse Kinematics using Tikhonov

Extra Credits:
- Implemented dual-quaternion skinning methods through adding a new class. Compare DQS method with LBS method (Linear Blend Skinning)
- Implement the pseudoinverse IK method. Compare pseudoinverse IK with Tikhonov regularization.
- When the user moves the IK handle for a long distance, divide the IK process into several sub-steps to improve the solution, where each sub-step solves the IK problem on a portion of the original distance.

Submission Requirement
- Source code
    - current folder
- CSCI_520_Inverse_Kinematics.pdf
- images/
    - contain 600 frames

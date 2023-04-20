CSCI 520, Assignment 3

Wanyu Zhang
USCID: 6773-4457-19

================
To graders:
This README.txt is used to describe the features and extra credits in a basic way.
For more details, please go to assignment report.

[Two Late Day policy Applied]

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

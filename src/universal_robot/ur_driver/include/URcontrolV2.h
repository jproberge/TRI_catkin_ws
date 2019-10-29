/**********************************************************************************************************************
 *
 *	Name		: URcontrolV2.h
 *	Author		: Deen Cockburn
 *	Date		: July 23 2015
 *	Version 	: v2.0 - Remove MoveIt and use only tf
 *
 *
 *	Description	: Header file for UR control using ROS
 *				  This library contains functions to calculate desired poses and compute trajectories.
 *
 **********************************************************************************************************************/



#include <tf/transform_datatypes.h>
#include <math.h>

# define ZERO_THRESH  0.000000001
# define PI           3.14159265358979323846  /* pi */




/**********************************************************************************************************************
 *
 *	Name		: rotaX
 *	Variables	: [in] 		angleRad 	- Angle in radian to turn around X axis
 *				  [in/out]	rotaXmatrix	- Homogeneous matrix with desired rotation matrix
 *
 *	Description	: This function will create a homogeneous matrix where the rotation matrix corresponds to a "angleRad"
 *				  rotation around the X axis.
 *
 **********************************************************************************************************************/
void rotaX(double angleRad, tf::Transform& rotaXmatrix);

/**********************************************************************************************************************
 *
 *	Name		: rotaY
 *	Variables	: [in] 		angleRad 	- Angle in radian to turn around Y axis
 *				  [in/out]	rotaYmatrix	- Homogeneous matrix with desired rotation matrix
 *
 *	Description	: This function will create a homogeneous matrix where the rotation matrix corresponds to a "angleRad"
 *				  rotation around the Y axis.
 *
 **********************************************************************************************************************/
void rotaY(double angleRad, tf::Transform& rotaYmatrix);

/**********************************************************************************************************************
 *
 *	Name		: rotaZ
 *	Variables	: [in] 		angleRad 	- Angle in radian to turn around Z axis
 *				  [in/out]	rotaZmatrix	- Homogeneous matrix with desired rotation matrix
 *
 *	Description	: This function will create a homogeneous matrix where the rotation matrix corresponds to a "angleRad"
 *				  rotation around the Z axis.
 *
 **********************************************************************************************************************/
void rotaZ(double angleRad, tf::Transform& rotaZmatrix);


/**********************************************************************************************************************
 *
 *	Name		: rotaXYZ
 *	Variables	: [in] 		angleRadX 		- Angle in radian to turn around X axis
 *				  [in] 		angleRadY 		- Angle in radian to turn around Y axis
 *				  [in] 		angleRadZ 		- Angle in radian to turn around Z axis
 *				  [in/out]	rotaXYZmatrix	- Homogeneous matrix with desired rotation matrix
 *
 *	Description	: This function will create a homogeneous matrix where the rotation matrix corresponds to a rotation
 *				  applied around the three axis in the order X-Y-Z.
 *
 **********************************************************************************************************************/
void rotaXYZ(double angleRadX, double angleRadY, double angleRadZ, tf::Transform& rotaXYZmatrix);


/**********************************************************************************************************************
 *
 *	Name		: rotaZYX
 *	Variables	: [in] 		angleRadZ 		- Angle in radian to turn around Z axis
 *				  [in] 		angleRadY 		- Angle in radian to turn around Y axis
 *				  [in] 		angleRadX 		- Angle in radian to turn around X axis
 *				  [in/out]	rotaZYXmatrix	- Homogeneous matrix with desired rotation matrix
 *
 *	Description	: This function will create a homogeneous matrix where the rotation matrix corresponds to a rotation
 *				  applied around the three axis in the order Z-Y-X.
 *
 **********************************************************************************************************************/
void rotaZYX(double angleRadZ, double angleRadY, double angleRadX, tf::Transform& rotaZYXmatrix);




/**********************************************************************************************************************
 *
 *	Name		: orientEffectorTowardsTable
 *	Variables	: [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_oriented	- Pose with corrected orientation to align the gripper with the table
 *
 *	Description	: This function will create a pose with the current cartesian position of the robot face plate but with
 *				  the correct orientation to have the gripper facing the table.
 *
 **********************************************************************************************************************/
void orientEffectorTowardsTable(tf::Transform& tfpose_current, tf::Transform& tfpose_oriented);



/**********************************************************************************************************************
 *
 *	Name		: rotateGripper
 *	Variables	: [in] 		rotationOrder	- Order in wich to apply rotations (options : "XYZ" and "ZYX")
 *				  [in] 		angleRadX 		- Angle in radian to turn around X axis
 *				  [in] 		angleRadY 		- Angle in radian to turn around Y axis
 *				  [in] 		angleRadZ 		- Angle in radian to turn around Z axis
 *				  [in]		gripperOffset	- Translation of selected tool frame from the robot face plate
 *				  [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_oriented	- Calculated pose
 *
 *	Description	: This function will create a pose of the face plate to correctly apply the desired rotation around
 *				  the specified tool frame.
 *
 **********************************************************************************************************************/
void rotateGripper(std::string rotationOrder, double angleRadX, double angleRadY, double angleRadZ,
        tf::Vector3 gripperOffset,tf::Transform& tfpose_current, tf::Transform& tfpose_oriented);


/**********************************************************************************************************************
 *
 *	Name		: rotateGripperX
 *	Variables	: [in] 		angleRad 		- Angle in radian to turn around X axis
 *				  [in]		gripperOffset	- Translation of selected tool frame from the robot face plate
 *				  [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_oriented	- Calculated pose
 *
 *	Description	: This function will create a pose of the face plate to correctly apply the desired rotation around
 *				  the specified tool frame.
 *
 **********************************************************************************************************************/
void rotateGripperX(double angleRad, tf::Vector3 gripperOffset,tf::Transform& tfpose_current,
        tf::Transform& tfpose_oriented);


/**********************************************************************************************************************
 *
 *	Name		: rotateGripperY
 *	Variables	: [in] 		angleRad 		- Angle in radian to turn around Y axis
 *				  [in]		gripperOffset	- Translation of selected tool frame from the robot face plate
 *				  [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_oriented	- Calculated pose
 *
 *	Description	: This function will create a pose of the face plate to correctly apply the desired rotation around
 *				  the specified tool frame.
 *
 **********************************************************************************************************************/
void rotateGripperY(double angleRad, tf::Vector3 gripperOffset,tf::Transform& tfpose_current,
        tf::Transform& tfpose_oriented);


/**********************************************************************************************************************
 *
 *	Name		: rotateGripperZ
 *	Variables	: [in] 		angleRad 		- Angle in radian to turn around Z axis
 *				  [in]		gripperOffset	- Translation of selected tool frame from the robot face plate
 *				  [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_oriented	- Calculated pose
 *
 *	Description	: This function will create a pose of the face plate to correctly apply the desired rotation around
 *				  the specified tool frame.
 *
 **********************************************************************************************************************/
void rotateGripperZ(double angleRad, tf::Vector3 gripperOffset,tf::Transform& tfpose_current,
        tf::Transform& tfpose_oriented);


/**********************************************************************************************************************
 *
 *	Name		: rotateGripperXYZ
 *	Variables	: [in] 		angleRadX 		- Angle in radian to turn around X axis
 *				  [in] 		angleRadY 		- Angle in radian to turn around Y axis
 *				  [in] 		angleRadZ 		- Angle in radian to turn around Z axis
 *				  [in]		gripperOffset	- Translation of selected tool frame from the robot face plate
 *				  [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_oriented	- Calculated pose
 *
 *	Description	: This function will create a pose of the face plate to correctly apply the desired rotation around
 *				  the specified tool frame. Rotation order X-Y-Z.
 *
 **********************************************************************************************************************/
void rotateGripperXYZ(double angleRadX, double angleRadY, double angleRadZ,tf::Vector3 gripperOffset,
        tf::Transform& tfpose_current, tf::Transform& tfpose_oriented);


/**********************************************************************************************************************
 *
 *	Name		: rotateGripperZYX
 *	Variables	: [in] 		angleRadZ 		- Angle in radian to turn around Z axis
 *				  [in] 		angleRadY 		- Angle in radian to turn around Y axis
 *				  [in] 		angleRadX 		- Angle in radian to turn around X axis
 *				  [in]		gripperOffset	- Translation of selected tool frame from the robot face plate
 *				  [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_oriented	- Calculated pose
 *
 *	Description	: This function will create a pose of the face plate to correctly apply the desired rotation around
 *				  the specified tool frame. Rotation order Z-Y-X.
 *
 **********************************************************************************************************************/
void rotateGripperZYX(double angleRadZ, double angleRadY, double angleRadX,tf::Vector3 gripperOffset,
        tf::Transform& tfpose_current, tf::Transform& tfpose_oriented);


/**********************************************************************************************************************
 *
 *	Name		: translateGripper
 *	Variables	: [in] 		axis 			- Axis in which translation must be applied (options : "X" "Y" "Z")
 *				  [in] 		distance 		- Desired distance in metres.
 *				  [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_translated	- Calculated pose
 *
 *	Description	: This function will create a pose of the face plate to correctly apply the desired translation in the
 *				  direction of "axis".
 *
 **********************************************************************************************************************/
void translateGripper(std::string axis,double distance, tf::Transform & tfpose_current,
                      tf::Transform & tfpose_translated);


/**********************************************************************************************************************
 *
 *	Name		: translateGripperX
 *	Variables	: [in] 		distance 		- Desired distance in metres.
 *				  [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_translated	- Calculated pose
 *
 *	Description	: This function will create a pose of the face plate to correctly apply the desired translation in the
 *				  direction of "X".
 *
 **********************************************************************************************************************/
void translateGripperX(double distance, tf::Transform & tfpose_current,
                       tf::Transform & tfpose_translated);


/**********************************************************************************************************************
 *
 *	Name		: translateGripperY
 *	Variables	: [in] 		distance 		- Desired distance in metres.
 *				  [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_translated	- Calculated pose
 *
 *	Description	: This function will create a pose of the face plate to correctly apply the desired translation in the
 *				  direction of "Y".
 *
 **********************************************************************************************************************/
void translateGripperY(double distance, tf::Transform & tfpose_current,
                       tf::Transform & tfpose_translated);


/**********************************************************************************************************************
 *
 *	Name		: translateGripperZ
 *	Variables	: [in] 		distance 		- Desired distance in metres.
 *				  [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_translated	- Calculated pose
 *
 *	Description	: This function will create a pose of the face plate to correctly apply the desired translation in the
 *				  direction of "Z".
 *
 **********************************************************************************************************************/
void translateGripperZ(double distance, tf::Transform & tfpose_current,
                       tf::Transform & tfpose_translated);


/**********************************************************************************************************************
 *
 *	Name		: tfSubvecnorm
 *	Variables	: [in] 		tfVec1	 		- Vectors to subtract
 *				  [in] 		tfVec2
 *				  [in/out]	UnitVector		- Normalised result of subtraction
 *
 *	Description	: This function will subtract tfVec1 from tfVec2 (tfVec2 - tfVec1) and normalise the result.
 *
 **********************************************************************************************************************/
void tfSubvecnorm(tf::Vector3 & tfVec1, tf::Vector3 & tfVec2, tf::Vector3& UnitVector);


/**********************************************************************************************************************
 *
 *	Name		: tfCrossproductnorm
 *	Variables	: [in] 		VectorA	 		- Vectors
 *				  [in] 		VectorB
 *				  [in/out]	vector			- Normalised result of cross product
 *
 *	Description	: This function will compute a cross product (VectorA X VectorB) and normalise the result.
 *
 **********************************************************************************************************************/
void tfCrossproductnorm(tf::Vector3 & VectorA, tf::Vector3 & VectorB, tf::Vector3& vector);


/**********************************************************************************************************************
 *
 *	Name		: slideGripperX
 *	Variables	: [in] 		distance	 	- Distance in metres to "slide"
 *				  [in] 		tfpose_current	- Current pose of the robot
 *				  [in/out]	tfpose_slide		- Calculated pose
 *
 *	Description	: This function will calculate the end pose of a sliding motion in the "X" direction (scooping motion).
 *				  In order to compute this function, Y axis of the gripper must be parallel to the table. If not,
 *				  pose_sile will be set to pose_init and an error message will be generated.
 *
 **********************************************************************************************************************/
void slideGripperX(double distance, tf::Transform & tfpose_current, tf::Transform & tfpose_slide);



/**********************************************************************************************************************
 *
 *	Name		: computeTargetPose
 *	Variables	: [in] 		object_angle 		- Angle returned by vision system
 *				  [in] 		tfpose_UR_CAM		- Tf homogeneous matrix of the camera frame relative to the robot basis.
 *				  								(calibration matrix)
 *				  [in] 		tfpose_CAM_OBJECT	- Tf homogeneous matrix of the object location relative to the
 *				  								camera frame.(Returned by vision)
 *				  [in/out]	tfpose_target			- Calculated pose
 *
 *	Description	: This function will calculate the target pose to reach the objects geometrical center with correct
 *				  orientation of the gripper. It is important to note that this pose represents the position of the face
 *				  plate of the robot.
 *
 **********************************************************************************************************************/
void computeTargetPose(double object_angle, tf::Transform & tfpose_UR_CAM, tf::Transform & tfpose_CAM_OBJECT,
        tf::Transform& tfpose_target);


/**********************************************************************************************************************
 *
 *	Name		: calculateEucledianDistance
 *	Variables	: [in] 		pose_start	 		- Poses
 *				  [in] 		pose_finish
 *				  [out]							- Distance in metres between pose_start and pose_finish
 *
 *	Description	: This function will calculate and return the euclidian distance between two poses.
 *
 **********************************************************************************************************************/
double calculateEuclidianDistance (geometry_msgs::Pose pose_start, geometry_msgs::Pose pose_finish);




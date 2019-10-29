/**********************************************************************************************************************
 *
 *	Name		: URcontrolV2.cpp
 *	Author		: Deen Cockburn
 *	Date		: July 23 2015
 *	Version 	: v2.0 - Remove MoveIt and use only tf
 *
 *
 *	Description	: Source file for UR control using ROS
 *				  This library contains functions to calculate desired poses.
 *
 **********************************************************************************************************************/


#include "URcontrolV2.h"



//Create a rotation matrix around X
void rotaX(double angleRad, tf::Transform& rotaXmatrix)
{

	tf::Matrix3x3 Orientation;
	tf::Vector3 Translation;
	double sinus,cosin;

	rotaXmatrix.setIdentity();

	cosin = cos(angleRad);
	sinus = sin(angleRad);

	if(fabs(cosin)<ZERO_THRESH)
		cosin = 0;
	if(fabs(sinus)<ZERO_THRESH)
		sinus = 0;


	Orientation.setValue(1,0,0,0,cosin,-sinus,0,sinus,cosin);
	Translation.setX(0); Translation.setY(0); Translation.setZ(0);

	rotaXmatrix.setBasis(Orientation);
	rotaXmatrix.setOrigin(Translation);



}



//Create a rotation matrix around Y
void rotaY(double angleRad, tf::Transform& rotaYmatrix)
{

	tf::Matrix3x3 Orientation;
	tf::Vector3 Translation;
	double sinus,cosin;

	rotaYmatrix.setIdentity();

	cosin = cos(angleRad);
	sinus = sin(angleRad);

	if(fabs(cosin)<ZERO_THRESH)
		cosin = 0;
	if(fabs(sinus)<ZERO_THRESH)
		sinus = 0;


	Orientation.setValue(cosin,0,sinus,0,1,0,-sinus,0,cosin);
	Translation.setX(0); Translation.setY(0); Translation.setZ(0);

	rotaYmatrix.setBasis(Orientation);
	rotaYmatrix.setOrigin(Translation);



}


//Create a rotation matrix around Z
void rotaZ(double angleRad, tf::Transform& rotaZmatrix)
{

	tf::Matrix3x3 Orientation;
	tf::Vector3 Translation;
	double sinus,cosin;

	rotaZmatrix.setIdentity();

	cosin = cos(angleRad);
	sinus = sin(angleRad);

	if(fabs(cosin)<ZERO_THRESH)
		cosin = 0;
	if(fabs(sinus)<ZERO_THRESH)
		sinus = 0;


	Orientation.setValue(cosin,-sinus,0,sinus,cosin,0,0,0,1);
	Translation.setX(0); Translation.setY(0); Translation.setZ(0);

	rotaZmatrix.setBasis(Orientation);
	rotaZmatrix.setOrigin(Translation);

}

//This function will create a homogeneous matrix where the rotation matrix corresponds to a rotation
// applied around the three axis in the order X-Y-Z.
void rotaXYZ(double angleRadX, double angleRadY, double angleRadZ, tf::Transform& rotaXYZmatrix)
{

	tf::Transform rotaXmatrix, rotaYmatrix, rotaZmatrix, rotaTemp;

	rotaX(angleRadX,rotaXmatrix);
	rotaY(angleRadY,rotaYmatrix);
	rotaZ(angleRadZ,rotaZmatrix);

	rotaTemp.mult(rotaXmatrix,rotaYmatrix);
	rotaXYZmatrix.mult(rotaTemp,rotaZmatrix);

}


//This function will create a homogeneous matrix where the rotation matrix corresponds to a rotation
//applied around the three axis in the order Z-Y-X.
void rotaZYX(double angleRadZ, double angleRadY, double angleRadX, tf::Transform& rotaZYXmatrix)
{

	tf::Transform rotaXmatrix, rotaYmatrix, rotaZmatrix, rotaTemp;

	rotaX(angleRadX,rotaXmatrix);
	rotaY(angleRadY,rotaYmatrix);
	rotaZ(angleRadZ,rotaZmatrix);

	rotaTemp.mult(rotaZmatrix,rotaYmatrix);
	rotaZYXmatrix.mult(rotaTemp,rotaXmatrix);

}



//This function will create a pose with the current cartesian position of the robot face plate but with
//the correct orientation to have the gripper facing the table.
void orientEffectorTowardsTable(tf::Transform& tfpose_current, tf::Transform& tfpose_oriented)
{

	tf::Transform tfpose_rotaYmatrix;

	// Align effector face towards the table
    rotaX(-PI, tfpose_rotaYmatrix);

    // Create target pose
    tfpose_oriented = tfpose_current;;

    tfpose_oriented.setBasis(tfpose_rotaYmatrix.getBasis());

    // Move up a little so the motion is not too jerky
    tfpose_oriented.getOrigin().setZ(tfpose_oriented.getOrigin().getZ() + 0.01);

}


//This function will create a pose of the face plate to correctly apply the desired rotation around
//the specified tool frame.
void rotateGripper(std::string rotationOrder, double angleRadX, double angleRadY, double angleRadZ,
        tf::Vector3 gripperOffset,tf::Transform & tfpose_current, tf::Transform& tfpose_oriented)
{

	double roll, pitch, yaw;
	tf::Vector3 offsetForward, offsetBackward;
    tf::Transform tfrotaMatrix, tfpose_temp, tfpose_temp2, tfpose_offset;

	//Create desired rotation matrix
	if (rotationOrder == "XYZ")
	{
        rotaXYZ(angleRadX, angleRadY, angleRadZ, tfrotaMatrix);
	}
	else if (rotationOrder == "ZYX")
	{
        rotaZYX(angleRadZ, angleRadY, angleRadX, tfrotaMatrix);
	}
	else
	{
		ROS_INFO("Error in rotateGripper");
        tfpose_oriented = tfpose_current;
		return;
	}

	//Create the necessary offsets for computation
	offsetForward = gripperOffset;
	offsetBackward.setX(-offsetForward.getX());
	offsetBackward.setY(-offsetForward.getY());
	offsetBackward.setZ(-offsetForward.getZ());


	//Create homogeneous matrix offset
	tfpose_offset.setIdentity();
	tfpose_offset.setOrigin(offsetForward);

	//Add tool offset to the current pose
	tfpose_temp.mult(tfpose_current,tfpose_offset);

	//Rotate
    tfpose_temp2.mult(tfpose_temp,tfrotaMatrix);

	//Create homogeneous matrix offset
	tfpose_offset.setOrigin(offsetBackward);

	//Remove tool offset
	tfpose_oriented.mult(tfpose_temp2,tfpose_offset);

	//Get roll pitch yaw
	tfpose_oriented.getBasis().getRPY(roll, pitch, yaw);

}



//This function will create a pose of the face plate to correctly apply the desired rotation around
//the specified tool frame.
void rotateGripperX(double angleRad, tf::Vector3 gripperOffset,tf::Transform& tfpose_current,
        tf::Transform& tfpose_oriented)
{

    rotateGripper("XYZ",angleRad,0,0,gripperOffset,tfpose_current,tfpose_oriented);

}

//This function will create a pose of the face plate to correctly apply the desired rotation around
//the specified tool frame.
void rotateGripperY(double angleRad, tf::Vector3 gripperOffset,tf::Transform& tfpose_current,
        tf::Transform& tfpose_oriented)
{
    rotateGripper("XYZ",0,angleRad,0,gripperOffset,tfpose_current,tfpose_oriented);

}

//This function will create a pose of the face plate to correctly apply the desired rotation around
//the specified tool frame.
void rotateGripperZ(double angleRad, tf::Vector3 gripperOffset,tf::Transform& tfpose_current,
        tf::Transform& tfpose_oriented)
{
    rotateGripper("XYZ",0,0,angleRad,gripperOffset,tfpose_current,tfpose_oriented);

}


//This function will create a pose of the face plate to correctly apply the desired rotation around
//the specified tool frame. Rotation order X-Y-Z.
void rotateGripperXYZ(double angleRadX, double angleRadY, double angleRadZ,tf::Vector3 gripperOffset,
        tf::Transform& tfpose_current, tf::Transform& tfpose_oriented)
{
    rotateGripper("XYZ",angleRadX,angleRadY,angleRadZ,gripperOffset,tfpose_current,tfpose_oriented);
}


//This function will create a pose of the face plate to correctly apply the desired rotation around
//the specified tool frame. Rotation order Z-Y-X.
void rotateGripperZYX(double angleRadZ, double angleRadY, double angleRadX,tf::Vector3 gripperOffset,
        tf::Transform& tfpose_current, tf::Transform& tfpose_oriented)
{
    rotateGripper("ZYX",angleRadX,angleRadY,angleRadZ,gripperOffset,tfpose_current,tfpose_oriented);
}



//This function will create a pose of the face plate to correctly apply the desired translation in the
//direction of "axis".
void translateGripper(std::string axis,double distance, tf::Transform & tfpose_current,
                      tf::Transform & tfpose_translated)
{

    tf::Transform tfpose_translation;

	//Create translation homogeneous matrix
	tfpose_translation.setIdentity();

	if (axis == "X")
	{
		tfpose_translation.getOrigin().setX(distance);
	}
	else if (axis == "Y")
	{
		tfpose_translation.getOrigin().setY(distance);
	}
	else if (axis == "Z")
	{
		tfpose_translation.getOrigin().setZ(distance);
	}
	else
	{
		ROS_INFO("Error in translateGripper");
        tfpose_translated = tfpose_current;
		return;
	}

    //Translate position
	tfpose_translated.mult(tfpose_current,tfpose_translation);

}


//This function will create a pose of the face plate to correctly apply the desired translation in the
//direction of "X".
void translateGripperX(double distance, tf::Transform & tfpose_current,
                       tf::Transform & tfpose_translated)
{
    translateGripper("X",distance, tfpose_current, tfpose_translated);
}


//This function will create a pose of the face plate to correctly apply the desired translation in the
//direction of "Y".
void translateGripperY(double distance, tf::Transform & tfpose_current,
                       tf::Transform & tfpose_translated)
{
    translateGripper("Y",distance, tfpose_current, tfpose_translated);
}


//This function will create a pose of the face plate to correctly apply the desired translation in the
//direction of "Z".
void translateGripperZ(double distance, tf::Transform & tfpose_current,
                       tf::Transform & tfpose_translated)
{
    translateGripper("Z",distance, tfpose_current, tfpose_translated);
}


//This function will subtract tfVec1 from tfVec2 (tfVec2 - tfVec1) and normalise the result.
void tfSubvecnorm(tf::Vector3 & tfVec1, tf::Vector3 & tfVec2, tf::Vector3 & UnitVector)
{
    //tf::Vector3 UnitVector;
    double distance, X, Y, Z;


    UnitVector.setX(tfVec2.getX()-tfVec1.getX());
    UnitVector.setY(tfVec2.getY()-tfVec1.getY());
    UnitVector.setZ(tfVec2.getZ()-tfVec1.getZ());

    distance=sqrt(pow(UnitVector.getX(),2)+pow(UnitVector.getY(),2)+pow(UnitVector.getZ(),2));

    X = UnitVector.getX()/distance;
    Y = UnitVector.getY()/distance;
    Z = UnitVector.getZ()/distance;

    UnitVector.setX(X);
    UnitVector.setY(Y);
    UnitVector.setZ(Z);

}


//This function will compute a cross product (VectorA X VectorB) and normalise the result.
void tfCrossproductnorm(tf::Vector3 & VectorA, tf::Vector3 & VectorB, tf::Vector3& vector)
{
	double distance,X,Y,Z;

    vector.setX((VectorA.getY()*VectorB.getZ())-(VectorB.getY()*VectorA.getZ()));
    vector.setY(-(VectorA.getX()*VectorB.getZ())+(VectorB.getX()*VectorA.getZ()));
    vector.setZ((VectorA.getX()*VectorB.getY())-(VectorA.getY()*VectorB.getX()));


    distance=sqrt(pow(vector.getX(),2)+pow(vector.getY(),2)+pow(vector.getZ(),2));

    X = vector.getX()/distance;
    Y = vector.getY()/distance;
    Z = vector.getZ()/distance;

    vector.setX(X);
    vector.setY(Y);
    vector.setZ(Z);

}



//This function will calculate the end pose of a sliding motion in the "X" direction (scooping motion).
//In order to compute this function, Y axis of the gripper must be parallel to the table. If not,
//pose_sile will be set to pose_init and an error message will be generated.
void slideGripperX(double distance, tf::Transform & tfpose_current, tf::Transform & tfpose_slide)
{

    tf::Vector3 Y, Xp, Zp;
    tf::Transform tfpose_temp;
	tf::Matrix3x3 temp_orient;

    if (tfpose_current.getBasis().getColumn(1).getZ() != 0)
	{
		ROS_INFO("CAN NOT SLIDE FROM THIS POSITION");
        tfpose_slide = tfpose_current;
		return;
	}

    Y = tfpose_current.getBasis().getColumn(1);
    Zp.setValue(0,0,-1);

    tfCrossproductnorm(Y,Zp,Xp);

    temp_orient.setValue(Xp.getX(),Y.getX(),Zp.getX(),Xp.getY(),Y.getY(),Zp.getY(),Xp.getZ(),Y.getZ(),Zp.getZ());


	tfpose_temp.setIdentity();
	tfpose_temp.setBasis(temp_orient);
	tfpose_temp.setOrigin(tfpose_current.getOrigin());

    translateGripperX(distance,tfpose_temp,tfpose_slide);

    tfpose_slide.setRotation(tfpose_current.getRotation());



}


//This function will calculate the target pose to reach the objects geometrical center with correct
//orientation of the gripper. It is important to note that this pose represents the position of the face
//plate of the robot.
void computeTargetPose(double object_angle, tf::Transform  & tfpose_UR_CAM, tf::Transform & tfpose_CAM_OBJECT,
        tf::Transform& tfpose_target)
{
	double CAMroll, CAMpitch, CAMyaw;
    tf::Transform tfpose_rotaZmatrix;
    tf::Transform tfpose_temp;

	//Get the orientation of the camera
	tfpose_UR_CAM.getBasis().getRPY(CAMroll,CAMpitch,CAMyaw);

	//Create rotation matrix to orient the gripper with the camera frame
    rotaZ((CAMyaw+object_angle), tfpose_rotaZmatrix);

	//Get the position of the part
	tfpose_temp.mult(tfpose_UR_CAM,tfpose_CAM_OBJECT);

    //Turn around Z axis to orient gripper
    tfpose_target.mult(tfpose_temp,tfpose_rotaZmatrix);

    //ROS_INFO("x: %f, y: %f, z: %f",tfpose_target.getOrigin().getX(),tfpose_target.getOrigin().getY(),tfpose_target.getOrigin().getZ());


}


//This function will calculate and return the euclidian distance between two poses.
double calculateEuclidianDistance (geometry_msgs::Pose pose_start, geometry_msgs::Pose pose_finish)
{

	return sqrt(pow(pose_finish.position.x-pose_start.position.x,2)+pow(pose_finish.position.y-pose_start.position.y,2)
			+pow(pose_finish.position.z-pose_start.position.z,2));

}


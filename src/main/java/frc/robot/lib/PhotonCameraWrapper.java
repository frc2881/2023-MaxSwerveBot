// Copyright (c) 2023 FRC Team 2881 - The Lady Cans
//
// Open Source Software; you can modify and/or share it under the terms of BSD
// license file in the root directory of this project.

package frc.robot.lib;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class PhotonCameraWrapper {
	private PhotonCamera m_photonCamera;
	private PhotonPoseEstimator m_photonPoseEstimator;
  private String m_cameraName;
	
	public PhotonCameraWrapper(
		String cameraName, 
		Transform3d robotToCamera,
		PoseStrategy poseStrategy,
		AprilTagFieldLayout aprilTagFieldLayout
	) {  
		m_cameraName = cameraName;
		m_photonCamera = new PhotonCamera(cameraName);
		m_photonPoseEstimator = new PhotonPoseEstimator(
			aprilTagFieldLayout,
			poseStrategy, 
			m_photonCamera, 
			robotToCamera
		); 
		m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}

	public String getCameraName(){
		return m_cameraName;
	}

	/**
	* @param estimatedRobotPose The current best guess at robot pose
	* @return A pair of the fused camera observations to a single Pose2d on the field, and the time
	*     of the observation. Assumes a planar field and the robot is always firmly on the ground
	*/
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
		m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
		return m_photonPoseEstimator.update();
	}

	public void dispose() {
		if (m_photonCamera.isConnected()) {
			//m_photonCamera.close(); // BUG: Photon Camera class currently throws an exception here
		}
	}
}
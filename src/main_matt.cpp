
#include <iostream>
#include "particle_filter/particle.h"
#include "particle_filter/paricle_filter.h"
#include "particle_filter/identity_motionmodel.h"
#include "particle_filter/articulation_marker_sensormodel.h"
#include "particle_filter/articulation_data_sensormodel.h"
#include "particle_filter/visualizer.h"

#include "articulation_model_msgs/ModelMsg.h"
#include "articulation_model_msgs/TrackMsg.h"

#include "pr2_lfd_utils/WMData.h"

#include "particle_filter/random.h"

#include "particle_filter/articulation_io_utils.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include "particle_filter/articulation_manip_sensor_action_model.h"

#include "particle_filter/handle_finder.h"

#include "particle_filter/action.h"

#include "particle_filter/action_generator.h"


#include "particle_filter/kernel_density_estimator.h"
#include <iostream>
#include <fstream>

articulation_model_msgs::TrackMsg data_track;

void trackCB(const articulation_model_msgs::TrackMsgConstPtr msg)
{
  data_track = *msg;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "particle_filter");
  Visualizer::getInstance()->init();


  // -------------------------------- sensor model ----------------

  MotionModel<ArticulationModelPtr>* motionModel = new IdentityMotionModel<ArticulationModelPtr>;
  SensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>* sensorModel;
  sensorModel = new ArtMarkerSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;

  SensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>* sensorModelForInit;
  sensorModelForInit = new ArtDataSensorModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg>;


  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(10, 10);
  Eigen::VectorXd u = Eigen::VectorXd::Ones(10);
  Eigen::MatrixXd motionNoiseCov = covariance / 10000000; //10000, 100000

  //orientation
//  motionNoiseCov(3,3) = motionNoiseCov(4,4) = motionNoiseCov(5,5) = 0.001;
  //rest
//  motionNoiseCov(6,6) = motionNoiseCov(7,7) = motionNoiseCov(8,8) = motionNoiseCov(9,9)  = 0.001;

  Eigen::MatrixXd sensorNoiseCov = covariance / 2;




  //---------------------------------- particle filter initalization ----------

  const int particles_number = 300; //500,200
  //params for real data
  ros::NodeHandle nh;
  //ros::Subscriber track_sub = nh.subscribe("marker_topic",1, trackCB);
  ros::Subscriber track_sub = nh.subscribe("marker_topic",1, trackCB);

  //------------------------------- track from recorded data --------------------

  articulation_model_msgs::ModelMsg model_msg;
  bool got_track = false;
  std::cout << "Listening for poses" << std::endl;
  while (ros::ok() && !got_track)
  {
    ros::spinOnce();
    if (!data_track.header.stamp.isZero())
    {
      model_msg.track = data_track;
      std::cout << "taking: " << model_msg.track.pose.size() << " poses" << std::endl;
      got_track = true;
    }
  }

ROS_INFO_STREAM("Making PF");
  ParticleFilter<ArticulationModelPtr> pf (particles_number, model_msg, *motionModel, motionNoiseCov, motionNoiseCov, motionNoiseCov);

ROS_INFO_STREAM("Making Actiongenerator");
  //ActionGenerator action_gen;
  //std::vector <tf::Vector3> generated_actions = action_gen.generateActionDirections(5, 3);
//  std::vector <tf::Vector3> generated_actions = action_gen.generateActionDirections(7, 7);


  //ActionPtr action(new Action);

  //  ------------------------------ particle filter loop ------------------------------

ROS_INFO_STREAM("Making other crap");
  ros::Rate r(2);
  uint loop_count = 1;
  tf::TransformListener tf_listener;
  tf::StampedTransform marker_static_to_marker;


  ROS_INFO_STREAM("loop starting");
  while (ros::ok())
  {

// ------------------ current marker measurement ------------------------------------
    try
    {

      //TODO: wait for transform?
      // static marker, moving marker
//      tf_listener.waitForTransform("ar_marker_1", "/ar_marker_2", ros::Time(0), ros::Duration(3.0));
//      tf_listener.lookupTransform("ar_marker_1", "/ar_marker_2", ros::Time(0), marker_static_to_marker);
//      tf_listener.waitForTransform("ar_marker_4", "/ar_marker_15", ros::Time(0), ros::Duration(3.0));
//      tf_listener.lookupTransform("ar_marker_4", "/ar_marker_15", ros::Time(0), marker_static_to_marker);
//      tf_listener.waitForTransform("ar_marker_9", "/ar_marker_12", ros::Time(0), ros::Duration(3.0));
//      tf_listener.lookupTransform("ar_marker_9", "/ar_marker_12", ros::Time(0), marker_static_to_marker);
      tf_listener.waitForTransform("ar_marker_6", "/ar_marker_7", ros::Time(0), ros::Duration(3.0));
      tf_listener.lookupTransform("ar_marker_6", "/ar_marker_7", ros::Time(0), marker_static_to_marker);

    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    for (std::vector <Particle <ArticulationModelPtr> >::iterator it = pf.particles.begin(); it!= pf.particles.end(); ++it)
    {
      it->state->setParam("current_pose_trans", marker_static_to_marker.getOrigin(), articulation_model_msgs::ParamMsg::PRIOR);
      it->state->setParam("current_pose_quat", marker_static_to_marker.getRotation(), articulation_model_msgs::ParamMsg::PRIOR);

      geometry_msgs::Pose pose_obs, pose_proj;
      V_Configuration q;
      tf::poseTFToMsg(marker_static_to_marker, pose_obs);
      it->state->getCurrentPoseProjected(pose_obs, pose_proj, q);
      tf::Transform tf_pose_proj;
      tf::poseMsgToTF(pose_proj,tf_pose_proj);

      // get tf_pose_proj in the current_pose frame

      it->state->setParam("current_proj_pose_trans", tf_pose_proj.getOrigin(), articulation_model_msgs::ParamMsg::PRIOR);
      it->state->setParam("current_proj_pose_quat", tf_pose_proj.getRotation(), articulation_model_msgs::ParamMsg::PRIOR);

      if (it->state->model_msg.name == "prismatic")
      {
        tf::Vector3 prismatic_dir;
        boost::shared_ptr<PrismaticModel> prismatic_model = boost::dynamic_pointer_cast< PrismaticModel > (it->state);
        prismatic_dir.setX(prismatic_model->axis_x);
        prismatic_dir.setY(prismatic_model->axis_y);
        prismatic_dir.setZ(prismatic_model->axis_z);
        prismatic_dir = prismatic_dir * marker_static_to_marker.getBasis();
        it->state->setParam("current_proj_pose_prismatic_dir_in_current", prismatic_dir, articulation_model_msgs::ParamMsg::PRIOR);
      }


      if (it->state->model_msg.name == "rotational")
      {
        tf::Vector3 rot_proj_dir;
        boost::shared_ptr<RotationalModel> rotational_model = boost::dynamic_pointer_cast< RotationalModel > (it->state);
        tf::Matrix3x3 rot_axis_m(rotational_model->rot_axis);
        tf::Vector3 rot_axis_z = rot_axis_m.getColumn(2);
        tf::Vector3 radius = tf_pose_proj.getOrigin() - rotational_model->rot_center;
        rot_proj_dir = radius.cross(rot_axis_z);

        rot_proj_dir = rot_proj_dir * marker_static_to_marker.getBasis();

        it->state->setParam("current_proj_pose_rot_dir", rot_proj_dir, articulation_model_msgs::ParamMsg::PRIOR);
      }
    }

// ----------------- propagate particles -------------------------------------------
    ROS_INFO_STREAM ("loop_count: " << loop_count);

    pf.propagate(pf.particles, u, motionNoiseCov, *motionModel);

//    if (loop_count > 1)
    Visualizer::getInstance()->publishParticles(pf.particles);
    ros::spinOnce();

    if (loop_count % 10 == 0 || loop_count == 1)
    {



// ----------------- marker correction step -------------------
      if (loop_count == 1)
      {
        ROS_INFO ("executing correction step");
        articulation_model_msgs::TrackMsg z;
        pf.correct<articulation_model_msgs::TrackMsg>(pf.particles, z, sensorNoiseCov, *sensorModelForInit);
        ROS_INFO ("Correction step executed.");

        if (!pf.normalize(pf.particles))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }

        if (!pf.stratifiedResample(particles_number, pf.particles))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }
        ROS_INFO ("PARTICLES AFTER INITIAL RESAMPLING, USING DEMONSTRATION DATA ONLY");
        pf.printStatistics(pf.particles);

      }

      {
        ROS_INFO ("executing correction step");
        articulation_model_msgs::TrackMsg z;

        //get current measurement
        geometry_msgs::Pose current_pose;
        tf::poseTFToMsg(marker_static_to_marker, current_pose);
        z.pose.push_back(current_pose);

        pf.correct<articulation_model_msgs::TrackMsg>(pf.particles, z, sensorNoiseCov, *sensorModel);
        ROS_INFO ("Correction step executed.");
      }

      pf.normalize(pf.particles, true);
      pf.sortParticles(pf.particles);
      Visualizer::getInstance()->publishParticles(pf.particles);


// ----------------- normalize and resample ------------------------------------
      ROS_INFO("Executing normalization and resampling step");
        if (!pf.normalize(pf.particles))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }

        if (!pf.stratifiedResample(particles_number, pf.particles))
        {
          ROS_ERROR ("no particles left, quiting");
          return -1;
        }
      ROS_INFO ("RESAMPLE STEP EXECUTED.");

      pf.printStatistics(pf.particles);

      pf.removeAddedParticleFlags(pf.particles);
      ROS_INFO("Normalization and resampling step executed");
    }
    r.sleep();
    ++loop_count;
  }





  delete motionModel;
  delete sensorModel;
  delete sensorModelForInit;
}


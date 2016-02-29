#include "particle_filter/articulation_manip_sensor_action_model.h"
#include "particle_filter/prismatic_model.h"
#include "particle_filter/rotational_model.h"
#include "particle_filter/random.h"

#define matt_testing true

template <class StateType, class ZType, class AType> ArtManipSensorActionModel<StateType, ZType, AType>::ArtManipSensorActionModel():
  scale(1.0), log_multiplier(1)
{
  exponencial_likelihood = false;
  linear_likelihood = false;
  quadratic_likelihood = false;
  quadratic_likelihood_with_zero = false; //why not just use an enum
  fancy_likelihood = true;
}


template <class StateType, class ZType, class AType> ArtManipSensorActionModel<StateType, ZType, AType>::~ArtManipSensorActionModel()
{
}


template <class StateType, class ZType, class AType> ZType ArtManipSensorActionModel<StateType, ZType, AType>::sense(const StateType &state, const Eigen::VectorXd &noise) const
{
}

template <class StateType, class ZType, class AType> double ArtManipSensorActionModel<StateType, ZType, AType>::senseLikelihood(const ZType &z, const AType& a,
                                                                                                                                const StateType &state,
                                                                                                                                const Eigen::MatrixXd &cov) const
{
}

template <class StateType, class ZType, class AType> double ArtManipSensorActionModel<StateType, ZType, AType>::senseLogLikelihood(const ZType &z, const AType& a,
                                                                                                                                const StateType &state,
                                                                                                                                const Eigen::MatrixXd &cov) const
{
}


template <> double ArtManipSensorActionModel<ArticulationModelPtr, int, ActionPtr>::senseLikelihood(const int &z,
                                                                                                   const ActionPtr& a,
                                                                                                   const ArticulationModelPtr &state,
                                                                                                   const Eigen::MatrixXd &cov) const
{
  return 0.0;
}
// z = 1 means it succeeded


template <> double ArtManipSensorActionModel<ArticulationModelPtr, int, ActionPtr>::senseLogLikelihood(const int &z,
                                                                                                      const ActionPtr& a,
                                                                                                      const ArticulationModelPtr &state,
                                                                                                      const Eigen::MatrixXd &cov) const
{
  bool updated_prob_calc = true;
  double loglikelihood = 0;
  double prob_end = 0;
  tf::Vector3 & action_vec = a->action_direction;
  tf::Vector3 & result_vec = a->result_vector;
  tf::Vector3 model_vec;
  tf::Vector3 expectation_vec;
  
  switch(state->model) { //doing calculations for all models at once, later
      case (RIGID): {
          model_vec = tf::Vector3(0,0,0);
          break;
      }
      case (PRISMATIC): {
          state->getParam("current_proj_pose_prismatic_dir_in_current", model_vec);
          break;
      }
      case (ROTATIONAL): {
          state->getParam("current_proj_pose_rot_dir", model_vec);
          break;
      }
      case (FREE): {
          model_vec = action_vec; //will always agree, right?
          break;
      }
  }
  
  model_vec.normalize();
  result_vec /= action_vec.length();
  action_vec.normalize();
  expectation_vec = model_vec*((model_vec.dot(action_vec))/(model_vec.dot(model_vec))); //project the action onto the model to create a realistic expectation vector for the model
  
  
  double action_result_relation = action_vec.dot(result_vec);
  double action_model_relation = action_vec.dot(model_vec);
  
  bool square_products = true;
  
  if (square_products) {
    action_result_relation = pow(action_result_relation,2);
    action_model_relation = pow(action_model_relation,2);
  }
  
  double agreement = 1.0-(expectation_vec-result_vec).length(); // projection minus the result
  agreement = (agreement<0)?0:agreement; //fix
  
  prob_end = (action_result_relation * action_model_relation) + ((1.0-action_result_relation) * (1.0-action_model_relation));
  prob_end *= agreement;
  
  
  
  loglikelihood = log(prob_end);
  
  
  /*
  switch (state->model)
  {
    case (RIGID):
      {
//        ROS_ERROR("RIGID");
        if (z == 1)
        {
          loglikelihood = std::numeric_limits<double>::lowest()/log_multiplier;
        }
        else
        {
          //TODO: how to increase likelihood here?
          loglikelihood = 0;
        }
        break;
      }

    case (PRISMATIC):
      {
//        ROS_ERROR("PRISMATIC");
        // calculate the relative angle between prismatic axis
        boost::shared_ptr<PrismaticModel> prismatic_model = boost::dynamic_pointer_cast< PrismaticModel > (state);

//        tf::Vector3 model_dir = prismatic_model->prismatic_dir;
        tf::Vector3 model_dir;
        state->getParam("current_proj_pose_prismatic_dir_in_current", model_dir);
        action_dir.normalize(); 
        //model_dir /= old_magnitude;
        
        //action_dir.normalize();
        //the result is scaled proportionately to the action direction, so don't normalize in quite the same way
        //result_dir /= old_magnitude; //maintains proportionality

        //double angle_rad = model_dir.angle(action_dir);
//        ROS_INFO("prismatic_proj_dir: x = %f, y = %f, z = %f;   action_dir: x = %f, y = %f, z = %f", (double)model_dir.getX(), (double)model_dir.getY(), (double)model_dir.getZ(), (double)action_dir.getX(), (double)action_dir.getY(), (double)action_dir.getZ());
//        ROS_INFO("angle_rad(degrees) unchanged : %f", angle_rad * 180/M_PI);
        
        // normalize the angle 0 < angle < 90
        double angle = angle_rad * 180/M_PI;
        int angle_int =  (int)angle % 180;
        angle_int = (angle_int + 180) % 180;
        if (angle_int > 90)
        {
          angle_int -= 180;
        }
        angle_rad = angle_int*M_PI/180;
        angle_rad = fabs(angle_rad); //Why?
        
        //while (angle < 0) {
        //    angle += (M_PI/(double)2.000);
        //}
        //fmod(angle,M_PI/(double)2.000);
        
//        ROS_INFO("angle_rad(degrees) : %f", angle_rad* 180/M_PI);
        
        //just take the square of the dot product and you'll get the number you want :)
        

        double model_action_relation = model_dir.dot(action_dir);
        double model_result_relation = model_dir.dot(result_dir);
        double action_result_relation = action_dir.dot(result_dir);
        
        bool square_products = true;
        
        if (square_products) {
            model_action_relation = pow(model_action_relation,2);
            model_result_relation = pow(model_result_relation,2);
            action_result_relation = pow(action_result_relation,2);
        }
        
        #if !matt_testing
            angle = action_result_relation*M_PI/2;
        #endif
        
        
        
        //double prob = 0;
        if(exponencial_likelihood)
        {
          prob = exp(-scale*angle_rad);
        }
        else if (linear_likelihood)
        {
          prob = -1.0/(M_PI/2.0) * angle_rad + 1.0;
        }
        else if (quadratic_likelihood)
        {
          prob = 4.0/(pow(M_PI, 2)) * pow(angle_rad - M_PI/2.0, 2);
        }
        else if (quadratic_likelihood_with_zero)
        {
          if (angle_rad > 40.0 * M_PI/180.0)
          {
            prob = 0;
          }
          else
          {
            prob = 4.0/(pow(M_PI, 2)) * pow(angle_rad - M_PI/2.0, 2);
          }
        }
        else if(fancy_likelihood) {
            
        }

        if (z == 1)
        {
          loglikelihood = log(prob);
          prob_end = prob;
        }
        else
        {
          loglikelihood = log(1.0-prob);
          prob_end = 1.0-prob;
        }
        break;
      }
    case (ROTATIONAL):
      {
        // calculate the relative angle between tangents
        boost::shared_ptr<RotationalModel> rotational_model = boost::dynamic_pointer_cast< RotationalModel > (state);
//        ROS_ERROR("ROTATIONAL");

        //get current pose_obs
        //geometry_msgs::Pose pose_obs, pose_proj;
        //tf::Quaternion quad_obs;
        //tf::Vector3 pos_obs;
        //state->getParam("current_pose_trans", pos_obs);
        //state->getParam("current_pose_quat", quad_obs);
        //tf::Transform tf_pose_obs(quad_obs, pos_obs);
        //tf::poseTFToMsg(tf_pose_obs, pose_obs);

        //project current pose_obs
        //V_Configuration q;
        //state->getCurrentPoseProjected(pose_obs, pose_proj, q);
        //tf::Transform tf_pose_proj;
        //tf::poseMsgToTF(pose_proj,tf_pose_proj);

        //get the tangent vector
        //tf::Matrix3x3 rot_axis_m(rotational_model->rot_axis);
        //tf::Vector3 rot_axis_z = rot_axis_m.getColumn(2);
        //tf::Vector3 radius = tf_pose_proj.getOrigin() - rotational_model->rot_center;
        //tf::Vector3 rot_proj_dir = radius.cross(rot_axis_z);
        //rot_proj_dir = rot_proj_dir * tf_pose_obs.getBasis();

        tf::Vector3 rot_proj_dir;
        state->getParam("current_proj_pose_rot_dir", rot_proj_dir);

        tf::Vector3 action_dir = a->action_direction;
        
        rot_proj_dir.normalize();
        action_dir.normalize();
        double angle_rad = rot_proj_dir.angle(action_dir);
//        ROS_INFO("rot_proj_dir: x = %f, y = %f, z = %f;   action_dir: x = %f, y = %f, z = %f", (double)rot_proj_dir.getX(), (double)rot_proj_dir.getY(), (double)rot_proj_dir.getZ(), (double)action_dir.getX(), (double)action_dir.getY(), (double)action_dir.getZ());
//        ROS_INFO("angle_rad(degrees) unchanged : %f", angle_rad * 180/M_PI);

        // normalize the angle 0 < angle < 90
       // double angle = angle_rad  180/M_PI;
        int angle_int =  (int)angle % 180;
        angle_int = (angle_int + 180) % 180;
        if (angle_int > 90)
        {
          angle_int -= 180;
        }
        angle_rad = angle_int*M_PI/180;
        angle_rad = fabs(angle_rad);
        
        //while (angle < 0) {
        //    angle += (M_PI/(double)2.000);
        //}
        //fmod(angle,M_PI/(double)2.000);


        double prob = 0;
        if(exponencial_likelihood)
        {
          prob = exp(-scale*angle_rad);
        }
        else if (linear_likelihood)
        {
          prob = -1.0/(M_PI/2.0) * angle_rad + 1;
//          ROS_INFO("angle_rad(degrees) : %f", angle_rad* 180/M_PI);
        }
        else if (quadratic_likelihood)
        {
          //prob = 4.0/(pow(M_PI, 2)) * pow(angle_rad - M_PI/2.0, 2);
         //prob =
        }
        else if (quadratic_likelihood_with_zero)
        {
          if (angle_rad > 40.0 * M_PI/180.0)
          {
            prob = 0;
          }
          else
          {
           // prob = 4.0/(pow(M_PI, 2)) * pow(angle_rad - M_PI/2.0, 2);
          }
        }


        if (z == 1)
        {
          loglikelihood = log(prob);
          prob_end = prob;
        }
        else
        {
          loglikelihood = log(1.0-prob);
          prob_end = 1.0-prob;
        }
        break;
      }
    case (FREE):
      {
        if (z == 1)
        {
          loglikelihood = 0;
        }
        else
        {
          loglikelihood = std::numeric_limits<double>::lowest()/log_multiplier;
        }
      }
  }*/
  
//  ROS_ERROR("prob = %f",prob_end);
//  ROS_ERROR("loglikelihood = %f",log_multiplier*loglikelihood);
  return log_multiplier*loglikelihood;
}



template class ArtManipSensorActionModel<ArticulationModelPtr, articulation_model_msgs::TrackMsg, ActionPtr>;
template class ArtManipSensorActionModel<ArticulationModelPtr, int, ActionPtr>;


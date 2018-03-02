#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include <rws2018_libs/team.h>
#include <rws2018_msgs/MakeAPlay.h>
#include <visualization_msgs/Marker.h>
#include <sstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace ros;
using namespace tf;

#define DEFAULT_TIME 0.05

float randomizePosition(float* x, float* y)
{
  srand(static_cast<unsigned>(time(0)));
  *x = (((static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 0.5) * 10);
  *y = (((static_cast<float>(rand()) / static_cast<float>(RAND_MAX)) - 0.5) * 10);
}

namespace rws_tmarques
{
class Player
{
public:
  // Constructor with the same name as the class
  Player(string name)
  {
    this->name = name;
  }

  int setTeamName(int team_index = 0 /*default value*/)
  {
    switch (team_index)
    {
      case 0:
        return setTeamName("red");
        break;
      case 1:
        return setTeamName("green");
        break;
      case 2:
        return setTeamName("blue");
        break;
      default:
        // cout << "wrong team index given. Cannot set team" << endl;
        ROS_ERROR("wrong team index given. Cannot set team");
        break;
    }
  }

  // Set team name, if given a correct team name (accessor)
  int setTeamName(string team)
  {
    if (team == "red" || team == "green" || team == "blue")
    {
      this->team = team;
      return 1;
    }
    else
    {
      this->team = "no team";
      // cout << "cannot set team name to " << team << endl;
      ROS_INFO("cannot set team name to %s", team.c_str());
      return 0;
    }
  }

  // Gets team name (accessor)
  string getTeam(void)
  {
    return team;
  }

  string name;  // A public atribute

private:
  string team;
};

class MyPlayer : public Player
{
public:
  boost::shared_ptr<Team> red_team;
  boost::shared_ptr<Team> green_team;
  boost::shared_ptr<Team> blue_team;

  boost::shared_ptr<Team> my_team;
  boost::shared_ptr<Team> my_preys;
  boost::shared_ptr<Team> my_hunter;

  ros::NodeHandle n;
  tf::TransformBroadcaster br;  // declare the bradcaster
  tf::Transform T;
  boost::shared_ptr<ros::Subscriber> sub;
  ros::Publisher vis_pub;
  tf::TransformListener listener;

  MyPlayer(string argin_name, string argin_team) : Player(argin_name)
  {
    vis_pub = n.advertise<visualization_msgs::Marker>("/bocas", 1);

    red_team = boost::shared_ptr<Team>(new Team("red"));
    green_team = boost::shared_ptr<Team>(new Team("green"));
    blue_team = boost::shared_ptr<Team>(new Team("blue"));

    if (red_team->playerBelongsToTeam(name))
    {
      my_team = red_team;
      my_preys = green_team;
      my_hunter = blue_team;
      setTeamName("red");
    }
    else if (green_team->playerBelongsToTeam(name))
    {
      my_team = green_team;
      my_preys = blue_team;
      my_hunter = red_team;
      setTeamName("green");
    }
    else if (blue_team->playerBelongsToTeam(name))
    {
      my_team = blue_team;
      my_preys = red_team;
      my_hunter = green_team;
      setTeamName("blue");
    }

    sub = boost::shared_ptr<ros::Subscriber>(new ros::Subscriber());
    *sub = n.subscribe("/make_a_play", 100, &MyPlayer::move, this);

    struct timeval t1;
    gettimeofday(&t1, NULL);
    srand(t1.tv_usec);
    double start_x = ((double)rand() / (double)RAND_MAX) * 10 - 5;
    double start_y = ((double)rand() / (double)RAND_MAX) * 10 - 5;
    printf("start_x=%f start_y=%f\n", start_x, start_y);
    warp(start_x, start_y, M_PI / 2);

    PrintReport();
  }

  void warp(double x, double y, double alpha)
  {
    T.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, alpha);
    T.setRotation(q);
    br.sendTransform(tf::StampedTransform(T, ros::Time::now(), "world", "tmarques"));
    ROS_INFO("Warping to x=%f y=%f, a=%f", x, y, alpha);
  }

  double getAngleToPlayer(string other_player, double time_to_wait = DEFAULT_TIME)
  {
    StampedTransform t;
    Time now = Time(0);
    try
    {
      listener.waitForTransform("tmarques", other_player, now, Duration(time_to_wait));
      listener.lookupTransform("tmarques", other_player, now, t);
    }
    catch (TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      return NAN;
    }
    return atan2(t.getOrigin().y(), t.getOrigin().x());
  }

  // distance
  float getDistanceToPlayer(string player_name, float time_to_wait = 0.1)
  {
    tf::StampedTransform trans;
    ros::Time now = Time(0);  // get the latest transform received

    try
    {
      listener.waitForTransform(name, player_name, now, Duration(time_to_wait));
      listener.lookupTransform(name, player_name, now, trans);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.01).sleep();
      return 10000;
    }

    float x = trans.getOrigin().x();
    float y = trans.getOrigin().y();
    return sqrt(x * x + y * y);
  }

  void move(const rws2018_msgs::MakeAPlay::ConstPtr& msg)
  {
    double x = T.getOrigin().x();
    double y = T.getOrigin().y();
    double a = 0;

    //--------------------------
    //----AI PART---------------
    //--------------------------
    // tf::StampedTransform mtiagoTrsilva;

    double displ = 6;

    double min_distance = 99999;
    string player_to_hunt = "no player";
    for (size_t i = 0; i < my_preys->player_names.size(); i++)
    {
      double dist = getDistanceToPlayer(my_preys->player_names[i]);
      if (isnan(dist))
      {
      }
      else if (dist < min_distance)
      {
        min_distance = dist;
        player_to_hunt = my_preys->player_names[i];
      }
    }
    double displacement = 1;  // max velocity for now

    double delta_alpha = getAngleToPlayer(player_to_hunt);

    if (isnan(delta_alpha))
      delta_alpha = 0;
    //--------------------------
    //----CONSTRAINS PART-------
    //--------------------------
    double displ_max = msg->dog;
    double displ_with_constrains;
    displ > displ_max ? displ = displ_max : displ = displ;

    double delta_alpha_max = M_PI / 30;

    // clang-format off
    fabs(delta_alpha) > fabs(delta_alpha_max) ? delta_alpha = delta_alpha_max * delta_alpha / fabs(delta_alpha) :                      delta_alpha = delta_alpha;
    // clang-format on
    tf::Transform my_move_T;
    my_move_T.setOrigin(tf::Vector3(displ, 0.0, 0.0));
    tf::Quaternion q1;
    q1.setRPY(0, 0, delta_alpha);
    my_move_T.setRotation(q1);

    T = T * my_move_T;
    br.sendTransform(tf::StampedTransform(T, ros::Time::now(), "world", "tmarques"));

    //-----Boca

    visualization_msgs::Marker marker;
    marker.header.frame_id = name;
    marker.header.stamp = ros::Time();
    marker.ns = "tmarques";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = "Gotta catch'em all!";

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.5;
    marker.pose.position.y = 0.5;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.z = 0.35;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.lifetime = ros::Duration(2);
    // only if using a MESH_RESOURCE marker type:
    // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub.publish(marker);
  }

  void PrintReport()
  {
    // cout << "My name is " << name << " and my team is " << getTeam() << endl;
    // substituir o cout por um print à ros
    // ROS_INFO_STREAM("My name is " << name << " and my team is " << getTeam())
    ROS_INFO("My name is %s and my team is %s ", name.c_str(), getTeam().c_str());
  }
};
}

int main(int argc, char** argv)
{
  // Creating an instance of class Player
  ros::init(argc, argv, "tmarques");

  rws_tmarques::MyPlayer my_player("tmarques", "green");

  if (my_player.red_team->playerBelongsToTeam("tmarques"))
  {
    // cout << "o tiago esta na equipa certa" << endl;
    ROS_INFO("O Tiago esta na equipa certa");
  };

  ros::NodeHandle n;

  ros::Rate loop_rate(10);
  // while (ros::ok())
  // {
  //   my_player.move();
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

  ros::spin();
}
#include <ros/ros.h>
#include <tf/transform_listener.h>   // transforms(座標変換)を簡単に受け取れるようにTransformListenerが実装されている
//#include <turtlesim/Velocity.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "my_tf_listener");

	ros::NodeHandle node;

	ros::service::waitForService("spawn");
	ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn srv;
	add_turtle.call(srv);

	ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

	tf::TransformListener listener; // オブジェクト生成

	ros::Rate rate(10.0);
	while(node.ok()) {
		tf::StampedTransform transform;
		try{
			ros::Time now = ros::Time::now();
			ros::Time past = ros::Time::now() - ros::Duration(5.0);
			//listener.waitForTransform("/turtle2", "/turtle1", past, ros::Duration(1.0)); // arguments1:取得したい変換のfromにあたるframe, 2:取得したい変換のtoにあたるframe, 3:指定する時間, 4:時間切れ:この最大の期間よりは長く待たない
			//listener.lookupTransform("/turtle2", "/turtle1", past, transform);  // arguments1:取得したいtransformのfromにあたるフレームを指定,2:取得したいtransformのtoにあたるフレームを指定,3:transformがしたい時間を指定する、ros::Time(0)を指定すると最新のtransformを取得できる,4:結果を取得したいobjectを指定
			listener.waitForTransform("/turtle2", now,
									  "/turtle1", past,
									  "/world", ros::Duration(1.0));
			listener.lookupTransform("/turtle2", now,
									 "/turtle1", past,
									 "/world", transform);
		}

		catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}

		geometry_msgs::Twist vel_msg;
		vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
		vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().y(), 2));
		// transformはturtle1からの距離や角度に基づいて、turtle2の直線速度と回転速度を計算するのに使われている
		turtle_vel.publish(vel_msg);
		// "turtle2/command_velocity"のtopicに新しい速度が配信され、シミュレータはこれによりturtle2の動きを更新する

		rate.sleep();
	}
	return 0;
};

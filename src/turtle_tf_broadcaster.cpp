#include <ros/ros.h>
#include <tf/transform_broadcaster.h>  // tfパッケージ
#include <turtlesim/Pose.h>

std::string turtle_name;



void poseCallback(const turtlesim::PoseConstPtr& msg) {
	static tf::TransformBroadcaster br;  // ケーブルを通して、変換を送るのに使うオブジェクト作成
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
	tf::Quaternion q;
	q.setRPY(0, 0, msg->theta); // 上３行で2Dのturtleのポーズから3d transformに情報をコピーする
	transform.setRotation(q); // 回転をセット
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name)); // 引数1:transform自体,2:現在の時間,timstampとして渡す,3:今作っているリンクの親フレームの名前を渡す,4:今作ってるリンクの子フレームを渡す
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "my_tf_broadcaster");
	if(argc != 2) { ROS_ERROR("need turtle name as argument"); return -1;};
	turtle_name = argv[1];

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

	ros::spin();
	return 0;
};

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <fstream>

int main(int argc, char** argv)
{

	int size=1000;
	char s;
	int counter=0;
	std::cout<<"in cpp file before"<<std::endl;
	ros::init(argc, argv, "camera_listener");
	ros::NodeHandle nh;
	ros::Duration(10.0).sleep();
	tf::TransformListener l;
	tf::TransformListener m;
	std::cout<<"in cpp file"<<std::endl;
	ros::Rate rate(10.0);
	std::fstream f;
	f.open("/home/anurag/sbr_workspace/src/sbr/assignment2/src/data.txt");




	while(nh.ok())
	{
		//std::cout<<"in cpp file"<<std::endl;
		std::cin>>s;
		tf::StampedTransform t1;
		tf::StampedTransform t2;
		try {
			l.waitForTransform("/base_link", "/ee_link", ros::Time(0), ros::Duration(10.0));
			l.lookupTransform("/base_link", "/ee_link", ros::Time(0), t1);
			m.waitForTransform("/camera_link", "/marker", ros::Time(0), ros::Duration(10.0));
			m.lookupTransform("/camera_link", "/marker", ros::Time(0), t2);
		}
		catch (tf::TransformException ex) {
			ROS_ERROR("Not found");
			ros::Duration(1.0).sleep();
			continue;
		}
		if (s=='x')
		{
			std::cout<<"in file case"<<std::endl;
			if(f.is_open())
			{
				f<<t1.getOrigin().x()<<" "<<t1.getOrigin().y() <<" "<<t1.getOrigin().z()<<" ";
				f<<t1.getRotation().x()<<" "<<t1.getRotation().y()<<" "<<t1.getRotation().z()<<" "<<t1.getRotation().w()<<std::endl;
			
				f<<t2.getOrigin().x()<<" "<<t2.getOrigin().y() <<" "<<t2.getOrigin().z()<<" ";
				f<<t2.getRotation().x()<<" "<<t2.getRotation().y()<<" "<<t2.getRotation().z()<<" "<<t2.getRotation().w()<<std::endl;
			}
			else
				std::cout<<"file not open"<<std::endl;
		}

		rate.sleep();
	}
	f.close();
	return 0;
	
};

#include"Detection.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Lane_Detection");
    ros::NodeHandle node;

    Detection detect(node);

//    ros::Rate loop_rate(10);
//    while(ros::ok())
//    {
////        auto start = std::chrono::high_resolution_clock::now();
////        if(detect.check == true)
////        {
////            detect.run();
////        }
////        auto end = std::chrono::high_resolution_clock::now();
////        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
////        std::cout << "time : " << duration.count() << " ms" << std::endl;
//        loop_rate.sleep();
//        ros::spinOnce();
//    }

    ros::spin();
    return 0;
}

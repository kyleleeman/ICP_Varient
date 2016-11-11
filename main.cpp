/*********************
 手动显示icp变体迭代过程
 *********************/

#include <iostream>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/console/parse.h>

using namespace std;
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> Cloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

string sources;
string targets;
bool next_iteration = false;
string icp_varient;

void showHelp(char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             ICP Varient Tutorial - Usage Guide             　　　　　　　 *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " source.pcd target.pcd --icp_varient(提供icp, gicp, nicp)" << std::endl << std::endl;
}

void parseCommandLine(int argc, char **argv)
{
  //显示帮助
  if(pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp(argv[0]);
    exit(0); 
  }
  
  //读取点云文件名
  std::vector<int> pcdnames;
  pcdnames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  if(pcdnames.size() != 2)
  {
    cout << "input pcd missing.\n";
    showHelp(argv[0]);
    exit(-1);
  }
  
  sources = argv[pcdnames[0]];
  targets = argv[pcdnames[1]];
}

//设置键盘交互函数
void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *nothing)
{
   if(event.getKeySym() == "space" && event.keyDown())
    next_iteration = true;
}

int main(int argc, char **argv)
{
  parseCommandLine(argc, argv);

  Cloud::Ptr src(new Cloud());
  Cloud::Ptr tar(new Cloud());
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (sources, *src) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file 1.pcd \n");
    return (-1);
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (targets, *tar) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file 2.pcd \n");
    return (-1);
  }
  
  //remove NAN points from the cloud (很重要！)
  std::vector<int> indices; 
  pcl::removeNaNFromPointCloud(*src, *src, indices); 
  pcl::removeNaNFromPointCloud(*tar, *tar, indices); 
  
  //读取icp名
  if(pcl::console::parse_argument (argc, argv, "--icp_varient", icp_varient) != -1)
  { 
    if(icp_varient.compare("icp") == 0)
    { 
      /******************registration********************/
      pcl::IterativeClosestPoint<Point, Point> icp;
      Cloud::Ptr icp_result (new Cloud);
      
      icp.setInputSource(src);
      icp.setInputTarget(tar);
      //忽略对应距离30cm之外的点
      icp.setMaxCorrespondenceDistance(0.1);
      icp.setTransformationEpsilon(1e-6);
      //前后两次迭代误差差值
      icp.setEuclideanFitnessEpsilon(0.1);
      //迭代次数
      icp.setMaximumIterations(2);
      icp_result = src;
      
      Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), T1;
      T1 <<  0.81993, -0.0028523, -0.57244 ,0.0341502,
	     -0.00229948,  0.999964, -0.00827393,  0.00155697,
	        0.572452 , 0.00809966  ,  0.819898  , 0.0381735,
	      0       ,    0       ,    0        ,   1;


      /********************display********************************/
      boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp test"));  //定义窗口共享指针
  
      int v1 ; //定义两个窗口v1，v2，窗口v1用来显示初始位置，v2用以显示配准过程
      int v2 ;
      
      view->createViewPort(0.0,0.0,0.5,1.0,v1);  //四个窗口参数分别对应x_min,y_min,x_max.y_max.
      view->createViewPort(0.5,0.0,1.0,1.0,v2);
	
      pcl::visualization::PointCloudColorHandlerCustom<Point> sources_cloud_color(src,250,0,0); //设置源点云的颜色为红色
      view->addPointCloud(src,sources_cloud_color,"sources_cloud_v1",v1);
      pcl::visualization::PointCloudColorHandlerCustom<Point> target_cloud_color (tar,0,250,0);  //目标点云为绿色
      view->addPointCloud(tar,target_cloud_color,"target_cloud_v1",v1); //将点云添加到v1窗口
      
      view->setBackgroundColor(0.0,0.05,0.05,v1); //设着两个窗口的背景色
      view->setBackgroundColor(0.05,0.05,0.05,v2);
      
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sources_cloud_v1");  //设置显示点的大小
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v1"); 
      
      pcl::visualization::PointCloudColorHandlerCustom<Point>aligend_cloud_color_icp(icp_result, 255,255,255);  //设置配准结果为白色
      view->addPointCloud(icp_result,aligend_cloud_color_icp,"aligend_cloud_v2",v2);
      view->addPointCloud(tar,target_cloud_color,"target_cloud_v2",v2);
      
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"aligend_cloud_v2");
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v2");
      
      view->registerKeyboardCallback(&keyboardEvent,(void*)NULL);  //设置键盘回调函数
      int iterations = 0; //迭代次数
      
      while(!view->wasStopped())
      {
	      view->spinOnce();  //运行视图
	      if (next_iteration)
	      {
		icp.setInputSource (src);
		icp.align (*icp_result, T1); //T1为给定的初值
		src = icp_result;
		
		cout <<"has conveged:"<<icp.hasConverged()<<"score:"<<icp.getFitnessScore()<<endl;
		cout<<"matrix:\n"<<icp.getFinalTransformation()<<endl;    //最终转换是结合T1之后的转换：final_transformation_ = transformation_ * final_transformation_;
		cout<<"iteration = "<<++iterations;
		cout << endl;
	        
		Ti = icp.getFinalTransformation() * Ti;
		cout << "Ti" << Ti << endl;
		
		/*... 如果icp.hasConverged=1,则说明本次配准成功，icp.getFinalTransformation()可输出变换矩阵   ...*/
		if (iterations == 1000)  //设置最大迭代次数
		  break;
        	view->updatePointCloud(icp_result,aligend_cloud_color_icp,"aligend_cloud_v2"); 
	      }
	      next_iteration = false;  //本次迭代结束，等待触发
      }
      
    }
    else if(icp_varient.compare("gicp") == 0)
    {
      /******************registration********************/
      pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
      gicp.setInputSource(src);
      gicp.setInputTarget(tar);
      gicp.setRotationEpsilon(0.1);
      gicp.setCorrespondenceRandomness(100);
      gicp.setMaximumIterations(2);
      Cloud::Ptr gicp_result(new Cloud());
      gicp_result = src;
      
      /******************display********************/
      boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp test"));  //定义窗口共享指针
  
      int v1 ; //定义两个窗口v1，v2，窗口v1用来显示初始位置，v2用以显示配准过程
      int v2 ;
      
      view->createViewPort(0.0,0.0,0.5,1.0,v1);  //四个窗口参数分别对应x_min,y_min,x_max.y_max.
      view->createViewPort(0.5,0.0,1.0,1.0,v2);
	
      pcl::visualization::PointCloudColorHandlerCustom<Point> sources_cloud_color(src,250,0,0); //设置源点云的颜色为红色
      view->addPointCloud(src,sources_cloud_color,"sources_cloud_v1",v1);
      pcl::visualization::PointCloudColorHandlerCustom<Point> target_cloud_color (tar,0,250,0);  //目标点云为绿色
      view->addPointCloud(tar,target_cloud_color,"target_cloud_v1",v1); //将点云添加到v1窗口
      
      view->setBackgroundColor(0.0,0.05,0.05,v1); //设着两个窗口的背景色
      view->setBackgroundColor(0.05,0.05,0.05,v2);
      
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sources_cloud_v1");  //设置显示点的大小
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v1"); 
      
      pcl::visualization::PointCloudColorHandlerCustom<Point>aligend_cloud_color_gicp(gicp_result, 255,255,255);  //设置配准结果为白色
      view->addPointCloud(gicp_result,aligend_cloud_color_gicp,"aligend_cloud_v2",v2);
      view->addPointCloud(tar,target_cloud_color,"target_cloud_v2",v2);
      
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"aligend_cloud_v2");
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v2");
      
      view->registerKeyboardCallback(&keyboardEvent,(void*)NULL);  //设置键盘回调函数
      int iterations = 0; //迭代次数
      
      while(!view->wasStopped())
      {
	      view->spinOnce();  //运行视图
	      if (next_iteration)
	      {
		gicp.setInputSource (src);
		gicp.align (*gicp_result);
		src = gicp_result;
		
		cout <<"has conveged:"<<gicp.hasConverged()<<"score:"<<gicp.getFitnessScore()<<endl;
		cout<<"matrix:\n"<<gicp.getFinalTransformation()<<endl;
		cout<<"iteration = "<<++iterations;
		cout << endl;
	        
		/*... 如果icp.hasConverged=1,则说明本次配准成功，icp.getFinalTransformation()可输出变换矩阵   ...*/
		if (iterations == 1000)  //设置最大迭代次数
		  break;
        	view->updatePointCloud(gicp_result,aligend_cloud_color_gicp,"aligend_cloud_v2"); 
	      }
	      next_iteration = false;  //本次迭代结束，等待触发
      }
    }
    else if(icp_varient.compare("nicp") == 0)
    {
      /******************registration********************/
      PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
      PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
      PointCloudWithNormals::Ptr reg_result (new PointCloudWithNormals);
      
      // Compute surface normals and curvature
      pcl::NormalEstimation<Point, PointNormalT> norm_est;
      
      pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
      norm_est.setSearchMethod (tree);
      norm_est.setKSearch (30);
      
      norm_est.setInputCloud (src);
      norm_est.compute (*points_with_normals_src);
      pcl::copyPointCloud (*src, *points_with_normals_src);

      norm_est.setInputCloud (tar);
      norm_est.compute (*points_with_normals_tgt);
      pcl::copyPointCloud (*tar, *points_with_normals_tgt);
      
      pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
      reg.setTransformationEpsilon (1e-6);
      // Set the maximum distance between two correspondences (src<->tgt) to 10cm
      // Note: adjust this based on the size of your datasets
      reg.setMaxCorrespondenceDistance (0.1);  
      reg.setInputSource (points_with_normals_src);
      reg.setInputTarget (points_with_normals_tgt);
      
      reg_result = points_with_normals_src;
      reg.setMaximumIterations (2);
      
      /********************display********************************/
      boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("gicp test"));  //定义窗口共享指针
  
      int v1 ; //定义两个窗口v1，v2，窗口v1用来显示初始位置，v2用以显示配准过程
      int v2 ;
      
      view->createViewPort(0.0,0.0,0.5,1.0,v1);  //四个窗口参数分别对应x_min,y_min,x_max.y_max.
      view->createViewPort(0.5,0.0,1.0,1.0,v2);
	
      pcl::visualization::PointCloudColorHandlerCustom<PointNormalT> sources_cloud_color(points_with_normals_src,250,0,0); //设置源点云的颜色为红色
      view->addPointCloud(points_with_normals_src,sources_cloud_color,"sources_cloud_v1",v1);
      pcl::visualization::PointCloudColorHandlerCustom<PointNormalT> target_cloud_color (points_with_normals_tgt,0,250,0);  //目标点云为绿色
      view->addPointCloud(points_with_normals_tgt,target_cloud_color,"target_cloud_v1",v1); //将点云添加到v1窗口
      
      view->setBackgroundColor(0.0,0.05,0.05,v1); //设着两个窗口的背景色
      view->setBackgroundColor(0.05,0.05,0.05,v2);
      
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"sources_cloud_v1");  //设置显示点的大小
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v1"); 
      
      pcl::visualization::PointCloudColorHandlerCustom<PointNormalT>aligend_cloud_color_nicp(reg_result, 255,255,255);  //设置配准结果为白色
      view->addPointCloud(reg_result,aligend_cloud_color_nicp,"aligend_cloud_v2",v2);
      view->addPointCloud(points_with_normals_tgt,target_cloud_color,"target_cloud_v2",v2);
      
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"aligend_cloud_v2");
      view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"target_cloud_v2");
      
      view->registerKeyboardCallback(&keyboardEvent,(void*)NULL);  //设置键盘回调函数
      int iterations = 0; //迭代次数
      
      while(!view->wasStopped())
      {
	      view->spinOnce();  //运行视图
	      if (next_iteration)
	      {
		reg.setInputSource (points_with_normals_src);
		reg.align (*reg_result);
		points_with_normals_src = reg_result;
		
		cout <<"has conveged:"<<reg.hasConverged()<<"score:"<<reg.getFitnessScore()<<endl;
		cout<<"matrix:\n"<<reg.getFinalTransformation()<<endl;
		cout<<"iteration = "<<++iterations;
		cout << endl;
	        
		/*... 如果icp.hasConverged=1,则说明本次配准成功，icp.getFinalTransformation()可输出变换矩阵   ...*/
		if (iterations == 1000)  //设置最大迭代次数
		  break;
        	view->updatePointCloud(reg_result,aligend_cloud_color_nicp,"aligend_cloud_v2"); 
	      }
	      next_iteration = false;  //本次迭代结束，等待触发

      }
     }
      else
      {
	std::cout << "Wrong icp name.\n";
	showHelp (argv[0]);
	exit (-1);
      }
    }

  return 0;
}
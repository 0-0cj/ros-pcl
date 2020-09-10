#include <iostream>
#include <string>
#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <vector>
#include <cmath>
#include <limits>

# define cj (numeric_limits<double>::max)()
using namespace std;

# define n_bin 38
# define z_diff 0.15
# define n_segment 180
# define r_1 0.2
# define r_2 0.5

# define r_max_square_2 50*50
# define point_to_line 0.3
# define sensor_hight 1.8
# define end_point 0.2
# define slope 0.3

# define min_angle 65.2
# define f_angle 0.6
# define PI 3.14159265358979323846

class Bin {
public:
	double min_z;
	double min_z_range;
	int id;
private:
	bool has_point_;
public:
	Bin() :has_point_(false) { init(); }
	void addPoint(const pcl::PointXYZ &point) {
		const double d = sqrt(point.x * point.x + point.y * point.y);
		addPoint(d, point.z);
	}
	void addPoint(const double &d, const double &z) {
		has_point_ = true;
	
		if (z < min_z) {
			min_z = z;
			min_z_range = d;
		}
	}

	bool hasPoint() { return has_point_; }
	void init() {
		min_z = cj;
	}
};

class Segment{
public:
	typedef std::pair<double, double> LocalLine;
	std::vector<Bin> bins_;
	std::list<LocalLine> lines_;
	std::vector<int> seed_inx_;
	std::vector<int> break_point_;
public:
	Segment() {
		bins_.resize(n_bin);
	};
	/*输入：点的高度、到原点的水平距离、所在bin格的编号
	  功能：判断这个点是否在拟合直线阈值范围内
	  输出：在返回true,不在返回false
	*/
	bool point_line(double d, double z, int bin_num) {
		if (break_point_.empty()) {
			if (lines_.empty()) {
				return true;
			}
			else {
				double hight = lines_.begin()->first*d + lines_.begin()->second;
				if (abs(z - hight) < point_to_line) {
					return false;
				}
				else return true;
			}
		}
		else {

			for (int i = 0; i < break_point_.size(); i++) {
				if (seed_inx_[break_point_[i]] >= bin_num) {
					list <LocalLine>::iterator it = lines_.begin();
					int j = 0;
					for (int j = 0; j < i; j++) {
						if (it != lines_.end()) {
							++it;
						}
					}
					double hight = it->first*d + it->second;
					if (abs(z - hight) < point_to_line) {
						return false;
					}
					else return true;
				}

			}

			double hight = lines_.back().first*d + lines_.back().second;
			if (abs(z - hight) < point_to_line) {
				return false;
			}
			else return true;

		}
	}

//通过条件判断选取种子点
std::vector<Bin> get_seed_points() {
		std::vector<Bin> current_line_points;
		for (int i = 0; i < bins_.size(); ++i) {
			if (current_line_points.empty()) {
				if (abs(bins_[i].min_z + sensor_hight) < z_diff + 0.15) {
					current_line_points.push_back(bins_[i]);
					seed_inx_.push_back(i);
				}
			}
			else {
				if (abs((bins_[i].min_z - current_line_points.back().min_z) / (bins_[i].min_z_range - current_line_points.back().min_z_range)) < slope) {
					current_line_points.push_back(bins_[i]);
					seed_inx_.push_back(i);
				}
			}
		}
		return current_line_points;
	}


	/*输入：一系列点
	  功能：最小二乘法拟合直线
	  输出：返回一条直线
	*/
	LocalLine fitLocalLine(const std::vector<Bin>& points) {
		const unsigned int n_points = points.size();

		if (n_points >= 2) {
			Eigen::MatrixXd X(n_points, 2);
			Eigen::VectorXd Y(n_points);
			unsigned int counter = 0;
			for (int i = 0; i < points.size(); ++i) {
				X(counter, 0) = points[i].min_z_range;
				X(counter, 1) = 1;
				Y(counter) = points[i].min_z;
				++counter;
			}
			Eigen::VectorXd result = X.colPivHouseholderQr().solve(Y);
			LocalLine line_result;
			line_result.first = result(0);
			line_result.second = result(1);
			return line_result;
		}
		else {
			LocalLine line_result;
			line_result.first = 0;
			line_result.second = 0;
			return line_result;
		}
	}
	/*输入：一系列点跟一条直线
	  功能：找到keypoint（找到第一个点到直线的距离超过阈值的点）
	  输出：返回keypoint所在的位置
	*/
	int subsection(const std::vector<Bin>& line, LocalLine &sub_lines_tem) {
		double b = line[0].min_z - sub_lines_tem.first*line[0].min_z_range;
		for (int i = 0; i < line.size(); ++i) {
			if (abs(line[i].min_z - (sub_lines_tem.first*line[i].min_z_range + b)) > end_point) {
				return i;
			}
		}
		return 0;
	}

	/*输入：一系列点
	  功能：拟合地面直线(一个递归函数)
	  输出：拟合后的地面直线
	*/
	void fitlines(std::vector<Bin> seed) {
		
		LocalLine lines_tem = fitLocalLine(seed);
		lines_.push_back(lines_tem);
		std::vector<Bin> seed_next1;
		std::vector<Bin> seed_next2;
		int middle_point = subsection(seed, lines_tem);
		if (break_point_.empty()) {
			if (middle_point != 0 && middle_point != n_bin) {
				break_point_.push_back(middle_point);
			}
		}
		else {
			//此处修改
			//if (middle_point != 0 && middle_point != n_bin && middle_point > 1) {
			if (middle_point != 0) {
				break_point_.push_back(middle_point + break_point_.back());
			}
		}

		if (middle_point != 0) {
			lines_.pop_back();
			for (int j = 0; j < middle_point; j++) {
				seed_next1.push_back(seed[j]);
			}
			for (int i = middle_point; i < seed.size(); i++) {
				seed_next2.push_back(seed[i]);
			}
			if (seed_next1.size() >= 2) {
				LocalLine lines_tem_1 = fitLocalLine(seed_next1);
				lines_.push_back(lines_tem_1);
			}
			if (seed_next2.size() > 2) {
				fitlines(seed_next2);
			}
			else {
				break_point_.pop_back();
			}
		}
	}

	//拟合当前这个Segment的地面线
	void fitSegmentLines() {
		std::vector<Bin> current_line_points = get_seed_points();
		if (current_line_points.size() > 0) {
			fitlines(current_line_points);
		}

	}

};

class GroundSegmentation {

public:
	GroundSegmentation() {
		segments_.resize(n_segment);
		pcl_sub = nh.subscribe("pcd_data", 10, &GroundSegmentation::cloud_cb, this);
	}


	int range_number(double range) {
		return int((atan(range / sensor_hight) * 180 / PI - min_angle) / f_angle);
	}
	/*输入：点云数据
	  功能：把每个点划分到属于自己的bin格
	  输出：构建好GroundSegmentation、Segment、Bin之间的关系
	*/
	void insertPoints() {
		bin_index_.resize(cloud_input.size());
		double segment_step = 2 * M_PI / n_segment;
		//double bin_step = (sqrt(r_max_square) - sqrt(r_min_square)) / n_bin;
		//double r_min = sqrt(r_min_square);

		for (int i = 0; i < cloud_input.size(); ++i) {
			if(cloud_input.points[i].x * cloud_input.points[i].x + cloud_input.points[i].y * cloud_input.points[i].y<r_max_square_2){
				double range_square = cloud_input.points[i].x * cloud_input.points[i].x + cloud_input.points[i].y * cloud_input.points[i].y;
				double range = sqrt(range_square);
				unsigned int bin_index = range_number(range);
				unsigned int segment_index = floor((std::atan2(cloud_input.points[i].y, cloud_input.points[i].x) + M_PI) / (2 * M_PI / n_segment));
				if (bin_index > n_bin - 1) { bin_index = n_bin - 1; }
				if (segment_index > n_segment - 1) { segment_index = segment_index % n_segment; }
				segments_[segment_index].bins_[bin_index].addPoint(cloud_input.points[i]);
				bin_index_[i] = std::make_pair(segment_index, bin_index);
			}
		}

	}
	//对GroundSegmentation拟合地面线
	void lineFit() {
		for (int i = 0; i < n_segment; ++i) {
			segments_[i].fitSegmentLines();
		}
	}
	

	//通过地面框架得到非地面点点云
	void object_output() {
		const double segment_step = 2 * M_PI / n_segment;
		for (int i = 0; i < cloud_input.size(); ++i) {
			if(cloud_input.points[i].x * cloud_input.points[i].x + cloud_input.points[i].y * cloud_input.points[i].y<r_max_square_2){
				double angle = std::atan2(cloud_input.points[i].y, cloud_input.points[i].x);
				unsigned int segment_index = (angle + M_PI) / segment_step;
				double range_square = cloud_input.points[i].x * cloud_input.points[i].x + cloud_input.points[i].y * cloud_input.points[i].y;
				double range = sqrt(range_square);
			
				unsigned int bin_index = range_number(range);
			
				if (bin_index > n_bin - 1) { bin_index = n_bin - 1; }
				if (segment_index > n_segment - 1) { segment_index = segment_index % n_segment; }

				if (segments_[segment_index].lines_.empty()) {
					cloud_object.push_back(cloud_input.points[i]);
				}
				else if (segments_[segment_index].lines_.begin()->first == 0 && segments_[segment_index].lines_.begin()->second == 0) {
					cloud_object.push_back(cloud_input.points[i]);
				}
				else {
					if (segments_[segment_index].point_line(range, cloud_input.points[i].z, bin_index)) {
						cloud_object.push_back(cloud_input.points[i]);
					}
					else {
					}
				}
			}
		}
	}
	
	void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
  		pcl::fromROSMsg (*input, cloud_input);
  		insertPoints();
  		lineFit();
  		object_output();
  		pcl::toROSMsg(cloud_object, output);
		pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("dealed_pcl", 1);
  		pcl_pub.publish (output);
		ROS_INFO("running...");
	}

	protected:
	vector<Segment> segments_;
	vector<std::pair<int, int> > bin_index_;
	pcl::PointCloud<pcl::PointXYZ> cloud_object;
	pcl::PointCloud<pcl::PointXYZ> cloud_input;
	ros::NodeHandle nh;
	ros::Subscriber pcl_sub;
	ros::Publisher pcl_pub;
	sensor_msgs::PointCloud2 output;
};


int 
main(int argc, char **argv){
    ros::init (argc, argv, "pcl_deal");
    GroundSegmentation G;
	ros::spin ();
	return 0;
}
#include <topic.hpp>

using namespace ros;
using namespace std;
using namespace std_msgs;
using namespace geometry_msgs;
//using namespace box_ros_msgs;
using namespace sensor_msgs;
using namespace cv;
using namespace  cv_bridge;

box_ box;
joint_states_ joint_states;
char direction;
float angle;
kbd_ros_msgs::kbd keyboard;
matrix_ matrix;
int image_width=0;
int image_height=0;

class SubscribeAndPublish
{
    public:
    SubscribeAndPublish()
    {
        //publish
        cmd_vel_pub=n_.advertise<Twist>("/cmd_vel",1);
        left_string_pub= n_.advertise<Float64>("/car/left_string_position_controller/command", 1);
        right_string_pub= n_.advertise<Float64>("/car/right_string_position_controller/command", 1);
        //subscribe
        joint_states_sub = n_.subscribe("/car/joint_states", 1, &SubscribeAndPublish::joint_states_callback, this);
        // image_sub=n_.subscribe("/car/camera/image", 1, &SubscribeAndPublish::image_callback, this);
        //imu_sub= n_.subscribe("/car/imu", 1, &SubscribeAndPublish::imu_callback, this);
        kbd_sub = n_.subscribe("/keyboard", 1, &SubscribeAndPublish::kbd_callback, this);

        box_msg_sub.subscribe(n_, "/yolov5/boxes", 1);
        odom_sub.subscribe(n_, "/odom", 1);

        sync.reset(new Sync(MySyncPolicy(10), box_msg_sub,odom_sub));
        sync->registerCallback(boost::bind(&box_odom_callback, _1, _2));
    }

    void image_callback(const ImageConstPtr & msg);
    void joint_states_callback(const JointStateConstPtr & msg);
    void imu_callback(const sensor_msgs::ImuConstPtr & msg);
    void kbd_callback(const kbd_ros_msgs::kbdConstPtr & msg);

    void input();
    private:
    NodeHandle n_;
    Publisher cmd_vel_pub;
    Publisher left_string_pub;
    Publisher right_string_pub;
    Subscriber joint_states_sub;
    Subscriber image_sub;
    Subscriber kbd_sub;
    //Subscriber imu_sub;
    message_filters::Subscriber<box_ros_msgs::BoundingBoxes> box_msg_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    typedef message_filters::sync_policies::ExactTime<box_ros_msgs::BoundingBoxes,nav_msgs::Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    Twist pub_cmd_vel;
    Float64 pub_left_string;
    Float64 pub_right_string;
};
void box_odom_callback(const box_ros_msgs::BoundingBoxesConstPtr & boxes,const nav_msgs::OdometryConstPtr & odom)
{
    //////////////////////////////////////////////odom//////////////////////////////////////////
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom->pose.pose.orientation,quat);
    double roll,pitch,yaw;
    tf::Matrix3x3(quat).getRPY(yaw,roll,pitch);
    pitch=-pitch-CV_PI/2;
    yaw=-yaw;
    //word坐标:左x;前y;上z（左手系）
    matrix.R = matrix_R(CV_PI/2+roll,pitch, yaw);
    //cout<<"roll:"<<roll<<endl<<"pitch:"<<pitch<<endl<<"yaw:"<<yaw<<endl<<endl;

    double sinp=sin(-pitch);
    double cosp=cos(-pitch);
    //cout<<sinp<<endl<<cosp<<endl<<endl;
    double x=odom->pose.pose.position.x;
    double y=odom->pose.pose.position.y;
    double x_=x*cosp+y*sinp;
    double y_=y*cosp-x*sinp;
    matrix.T.at<double>(0,0)=x_*1000;
    matrix.T.at<double>(1,0)=-1.1*1000;
    matrix.T.at<double>(2,0)=(y_)*1000;
    //////////////////////////////////////////////box//////////////////////////////////////////
    int count_y=0,count_r=0,count_b=0;
    int rx_sum=0,bx_sum=0;
    static vector<cv::Point> point;
    box.count.sum=boxes->count;
    for(int i=0;i<box.count.sum;i++)
    {
        int xmin=boxes->bounding_boxes[i].xmin;
        int xmax=boxes->bounding_boxes[i].xmax;
        int ymin=boxes->bounding_boxes[i].ymin;
        int ymax=boxes->bounding_boxes[i].ymax;
        cv::Point P2;
        cv::Point3d point_word;

        // int area=(xmax-xmin)*(ymax-ymin);
        // if(area<180)
        // {
        //     continue;
        // }

        if(boxes->bounding_boxes[i].Class[0]=='y')
        {
            P2=cv::Point((xmax+xmin)/2,ymax);
            point_word = Towordpoint(P2,0);
        }
        else
        {
            P2=cv::Point((xmax+xmin)/2,ymin);
            point_word = Towordpoint(P2,300);
        }

        if(boxes->bounding_boxes[i].Class[0]=='r')
        {
            box.red[count_r].name='r';
            box.red[count_r].id=count_r;
            box.red[count_r].point.x=(xmax+xmin)/2;
            box.red[count_r].point.y=(ymax+ymin)/2;
            box.red[count_r].Point3=point_word;
            box.red[count_r].point_top=P2;
            rx_sum+=box.red[count_r].point.x;
            count_r++;
        }
        else if(boxes->bounding_boxes[i].Class[0]=='b')
        {
            box.blue[count_b].name='b';
            box.blue[count_b].id=count_b;
            box.blue[count_b].point.x=(xmax+xmin)/2;
            box.blue[count_b].point.y=(ymax+ymin)/2;
            box.blue[count_b].Point3=point_word;
            box.blue[count_b].point_top=P2;
            bx_sum+= box.blue[count_b].point.x;
            count_b++;
        }
        else if(boxes->bounding_boxes[i].Class[0]=='y')
        {
            box.yellow[count_y].name='y';
            box.yellow[count_y].id=count_y;
            box.yellow[count_y].point.x=(xmax+xmin)/2;
            box.yellow[count_y].point.y=(ymax+ymin)/2;
            box.yellow[count_y].Point3=point_word;
            box.yellow[count_y].point_top=P2;
            count_y++;
        }
    }
    box.count.blue=count_b;
    box.count.red=count_r;
    box.count.yellow=count_y;

    if(rx_sum/count_r>bx_sum/count_b)
        box.sign=1;
    else
        box.sign=0;
    //根据纵坐标排序
    // box = box_sort(box);

    /////////////////////////////////////////////////image///////////////////////////////////////////////////
    static int init_sign=0;
    static CvImagePtr cv_ptr;
    static Mat src(boxes->img.height,boxes->img.width,CV_8UC3);
    static Mat word(boxes->img.height,boxes->img.width,CV_8UC3);
    double x_convert=1.0*boxes->img.width/160000;
    double y_convert=1.0*boxes->img.height/100000;
    vector<cv::Point> blue_point;
    vector<cv::Point> red_point;
    vector<cv::Point> road_point;
    vector<cv::Point3d> blue_point_3;
    vector<cv::Point3d> red_point_3;
    vector<cv::Point3d> road_point_3;
    if(init_sign==0)
    {
        image_width=boxes->img.width;
        image_height=boxes->img.height;
        namedWindow("img",WINDOW_FREERATIO);
        namedWindow("word",WINDOW_FREERATIO);
        init_sign=1;
    }
    sensor_msgs::Image img0;
    img0.data=boxes->img.data;
    img0.encoding=boxes->img.encoding;
    img0.height=boxes->img.height;
    img0.is_bigendian=boxes->img.is_bigendian;
    img0.step=boxes->img.step;
    img0.width=boxes->img.width;
    const sensor_msgs::Image& source=img0;
    cv_ptr = toCvCopy(source, "bgr8");
    src=cv_ptr->image;

    // if(box.count.red>3){
    //     box.count.red=3;
    // }

    // if(box.count.blue>3){
    //     box.count.blue=3;
    // }

    // if(box.sign==1)
    // {
    //     box.red[box.count.red].point=cv::Point(boxes->img.width,boxes->img.height);
    //     box.blue[box.count.blue].point=cv::Point(0,boxes->img.height);
    // }
    // else
    // {
    //     box.red[box.count.red].point=cv::Point(0,boxes->img.height);
    //     box.blue[box.count.blue].point=cv::Point(boxes->img.width,boxes->img.height);
    // }

    for(int i =0;i<box.count.red;i++)
    {
        circle(src,box.red[i].point,10,Scalar(255,0,0),CV_FILLED,CV_AA);
        red_point.push_back(box.red[i].point);
        int sign=0;
        if(red_point_3.size()==0&&box.count.red!=0)
        {
            red_point_3.push_back(box.red[0].Point3);
        }
        for(int j=0;j<red_point_3.size();j++)
        {
            if(!test_point(red_point_3[j] ,  box.red[i].Point3))
            {
                sign=1;
                break;
            }
        }

        if(sign)
        {
            red_point_3.push_back(box.red[i].Point3);
        }
    }

    for(int i=0;i<box.count.blue;i++)
    {
        circle(src,box.blue[i].point_top,5,Scalar(0,0,255),CV_FILLED,CV_AA);
        blue_point.push_back(box.blue[i].point);

        int sign=0;
        if(blue_point_3.size()==0&&box.count.blue!=0)
        {
            blue_point_3.push_back(box.blue[0].Point3);
        }
        for(int j=0;j<blue_point_3.size();j++)
        {
            if(!test_point(blue_point_3[j] ,  box.blue[i].Point3))
            {
                sign=1;
                break;
            }
        }

        if(sign)
        {
            blue_point_3.push_back(box.blue[i].Point3);
        }
    }

    for(int i=0;i<box.count.yellow;i++)
    {
        circle(src,box.yellow[i].point,10,Scalar(255,255,255),CV_FILLED,CV_AA);
    }

    for(int i=0;i<red_point_3.size();i++)
    {
        double x = boxes->img.width/2-x_convert*red_point_3[i].x;
        double y = boxes->img.height/2-y_convert*red_point_3[i].y;
        cv::Point P=cv::Point(x,y);
        //cout<<red_point_3[i]<<P<<endl;
        circle(word,P,1,Scalar(0,0,255),CV_FILLED,CV_AA);
    }

    for(int i=0;i<blue_point_3.size();i++)
    {
        double x = boxes->img.width/2-x_convert*blue_point_3[i].x;
        double y = boxes->img.height/2-y_convert*blue_point_3[i].y;
        cv::Point P=cv::Point(x,y);
        circle(word,P,1,Scalar(255,0,0),CV_FILLED,CV_AA);
    }

    // car_ctrl_ car_state=car_ctrl(direction,0);

    // cmd_vel_pub.publish(car_state.pub_cmd_vel);
    // left_string_pub.publish(car_state.pub_left_string);
    // right_string_pub.publish(car_state.pub_right_string);

    // circle(src, cv::Point(image_width/2,image_height*0.75), 20, Scalar(0, 0, 0), CV_FILLED, CV_AA);
    imshow("img",src);
    imshow("word",word);
    cv::waitKey(1);
}

void SubscribeAndPublish::kbd_callback(const kbd_ros_msgs::kbdConstPtr & msg)
{
    keyboard.w=msg->w;
    keyboard.a=msg->a;
    keyboard.s=msg->s;
    keyboard.d=msg->d;
    car_ctrl_ car_state=car_ctrl(direction,0);

    cmd_vel_pub.publish(car_state.pub_cmd_vel);
    left_string_pub.publish(car_state.pub_left_string);
    right_string_pub.publish(car_state.pub_right_string);
}

void SubscribeAndPublish::joint_states_callback(const JointStateConstPtr & msg)
{
    int i,j;
    static int sign=0,order[joint_NUM];
    if(sign==0)
    {
        char name[joint_NUM][36];
        for(i=0;i<joint_NUM;i++)
        {
            for(j=0;msg->name[i][j]!=0;j++)
            {
                name[i][j]=msg->name[i][j];
            }
            name[i][j]='\0';
        }

        for(j=0;j<joint_NUM;j++)
        {
            i=0;
            while (strcmp(name[i],joint_states.name[j]))
            {
                i++;
            }
            order[j]=i;
        }
        sign=1;
    }

    for(i=0;i<joint_NUM;i++)
    {
        joint_states.position[i]=msg->position[order[i]];
        joint_states.velocity[i]=msg->velocity[order[i]];
        joint_states.effort[i]=msg->effort[order[i]];
    }

}

// mono8: CV_8UC1, grayscale image
// mono16: CV_16UC1, 16-bit grayscale image
// bgr8: CV_8UC3, color image with blue-green-red color order
// rgb8: CV_8UC3, color image with red-green-blue color order
// bgra8: CV_8UC4, BGR color image with an alpha channel
// rgba8: CV_8UC4, RGB color image with an alpha channel
void SubscribeAndPublish::image_callback(const ImageConstPtr & msg)
{
    static int a=0;
    static CvImagePtr cv_ptr;
    static Mat src(msg->height,msg->width,CV_8UC3);
    static Mat word(msg->height,msg->width,CV_8UC3);
    double x_convert=1.0*msg->width/160000;
    double y_convert=1.0*msg->height/100000;
    vector<cv::Point> blue_point;
    vector<cv::Point> red_point;
    vector<cv::Point> road_point;
    vector<cv::Point3d> blue_point_3;
    vector<cv::Point3d> red_point_3;
    vector<cv::Point3d> road_point_3;
    if(a==0)
    {
        image_width=msg->width;
        image_height=msg->height;
        namedWindow("img",WINDOW_FREERATIO);
        namedWindow("word",WINDOW_FREERATIO);
        a=1;
    }
    cv_ptr = toCvCopy(msg, "bgr8");
    src=cv_ptr->image;

    // if(box.count.red>3){
    //     box.count.red=3;
    // }

    // if(box.count.blue>3){
    //     box.count.blue=3;
    // }

    // if(box.sign==1)
    // {
    //     box.red[box.count.red].point=cv::Point(msg->width,msg->height);
    //     box.blue[box.count.blue].point=cv::Point(0,msg->height);
    // }
    // else
    // {
    //     box.red[box.count.red].point=cv::Point(0,msg->height);
    //     box.blue[box.count.blue].point=cv::Point(msg->width,msg->height);
    // }

    for(int i =0;i<box.count.red;i++)
    {
        circle(src,box.red[i].point,10,Scalar(255,0,0),CV_FILLED,CV_AA);
        // char s[10];
        // sprintf(s,"%d",i);
        // putText(src,s,box.red[i].point,cv::FONT_HERSHEY_COMPLEX,2,Scalar(255,0,0),2);
        red_point.push_back(box.red[i].point);
        int sign=0;
        if(red_point_3.size()==0&&box.count.red!=0)
        {
            red_point_3.push_back(box.red[0].Point3);
        }
        for(int j=0;j<red_point_3.size();j++)
        {
            if(!test_point(red_point_3[j] ,  box.red[i].Point3))
            {
                sign=1;
                break;
            }
        }

        if(sign)
        {
            red_point_3.push_back(box.red[i].Point3);
        }
    }

    for(int i=0;i<box.count.blue;i++)
    {
        // char s[10];
        // sprintf(s,"%d",i);
        // putText(src,s,box.blue[i].point,cv::FONT_HERSHEY_COMPLEX,2,Scalar(255,0,0),2);
        circle(src,box.blue[i].point_top,5,Scalar(0,0,255),CV_FILLED,CV_AA);
        //circle(src,cv::Point(100,500),10,Scalar(0,0,255),CV_FILLED,CV_AA);
        blue_point.push_back(box.blue[i].point);

        int sign=0;
        if(blue_point_3.size()==0&&box.count.blue!=0)
        {
            blue_point_3.push_back(box.blue[0].Point3);
        }
        for(int j=0;j<blue_point_3.size();j++)
        {
            if(!test_point(blue_point_3[j] ,  box.blue[i].Point3))
            {
                sign=1;
                break;
            }
        }

        if(sign)
        {
            blue_point_3.push_back(box.blue[i].Point3);
        }
    }

    for(int i=0;i<box.count.yellow;i++)
    {
        // char s[10];
        // sprintf(s,"%d",i);
        // putText(src,s,box.yellow[i].point,cv::FONT_HERSHEY_COMPLEX,2,Scalar(255,0,0),2);
        circle(src,box.yellow[i].point,10,Scalar(255,255,255),CV_FILLED,CV_AA);
    }

    for(int i=0;i<red_point_3.size();i++)
    {
        double x = msg->width/2-x_convert*red_point_3[i].x;
        double y = msg->height/2-y_convert*red_point_3[i].y;
        cv::Point P=cv::Point(x,y);
        //cout<<red_point_3[i]<<P<<endl;
        circle(word,P,1,Scalar(0,0,255),CV_FILLED,CV_AA);
    }

    for(int i=0;i<blue_point_3.size();i++)
    {
        double x = msg->width/2-x_convert*blue_point_3[i].x;
        double y = msg->height/2-y_convert*blue_point_3[i].y;
        cv::Point P=cv::Point(x,y);
        circle(word,P,1,Scalar(255,0,0),CV_FILLED,CV_AA);
    }

    // cv::Point P=cv::Point(269,785);
    // circle(src,P,10,Scalar(255,0,0),CV_FILLED,CV_AA);
    // cv::Point3d point_word=Towordpoint(P.x,P.y,0);
    // cout<<point_word<<endl;

    // YAML::Node point_node = YAML::LoadFile("/home/wust/Personal_Data/ros_ws/robot-my_ws/src/robot/robot_program/config/point.yaml");
    // int p_x=point_node["point0"]["x"].as<std::int64_t>();
    // int p_y=point_node["point0"]["y"].as<std::int64_t>();
    // int p_z=point_node["point0"]["z"].as<std::int64_t>();

    // cv::Point P_=Touvpoint(p_x,p_y,p_z);
    // circle(src,P_,10,Scalar(255,0,0),CV_FILLED,CV_AA);
    // cout<<P_<<endl;
    // int n=2;
    // Mat mat_r = polyfit(red_point, n ,'y');
    // Mat mat_b = polyfit(blue_point, n, 'y');

    // for (int i =msg->height*2/4; i <msg->height; i++)
	// {
	// 	cv::Point red,blue,road;
	// 	red.y = i;
	// 	red.x = 0;

    //     blue.y = i;
	// 	blue.x = 0;

    //     road.y = i;
	// 	road.x = 0;

	// 	for (int j = 0; j < n + 1; ++j)
	// 	{
	// 		red.x += mat_r.at<double>(j, 0)*pow(i,j);
    //         blue.x += mat_b.at<double>(j, 0)*pow(i,j);
	// 	}
    //     road.x=(red.x+blue.x)/2;
    //     road_point.push_back(road);
	// 	circle(src, cv::Point(red.x,red.y), 5, Scalar(0, 0, 255), CV_FILLED, CV_AA);
	// 	circle(src, cv::Point(blue.x,blue.y), 5, Scalar(255, 0, 0), CV_FILLED, CV_AA);
	// 	circle(src, road, 5, Scalar(0, 0, 0), CV_FILLED, CV_AA);
	// }

    // Mat mat_road = polyfit(road_point, n, 'y');

    // cv::Point line1,line2;
    // line1=cv::Point(msg->width/2,msg->height);

    // double k=0;
    // for (int j = 0; j < n + 1; j++)
    // {
    //     k += j*mat_road.at<double>(j, 0)*pow(line1.y,j-1);
    // }

    // line2.y=msg->height*2/3;
    // line2.x=k*(line2.y-line1.y)+line1.x;

    // line(src,line1,line2,Scalar(0,255,0),10,CV_AA);

    // k=-1.0/k;
    // direction= k>0 ? 'L' : 'R';
    // angle = 90-fabs(atan(k)*180/CV_PI);

    // int y_=msg->height;
    // double y_1=0;
    // double y_2=0;
    // for (int j = 0; j < n + 1; j++)
    // {
    //     y_1 += j*mat_road.at<double>(j, 0)*pow(y_,j-1);
    // }
    // for (int j = 2; j <= n; j++)
    // {
    //     y_2 += factorial(j)*mat_road.at<double>(j, 0)*pow(y_,j-2);
    // }
    // double R=pow(1+pow(y_1,2),1.5)/y_2;
    // //circle(src,Point(msg->width,msg->height),(int)fabs(R),Scalar(255,255,255),5,LINE_AA);
    // //printf("%.2f\n",R);
    car_ctrl_ car_state=car_ctrl(direction,0);
    // // printf("%c,%.2f\n",direction,angle);

    cmd_vel_pub.publish(car_state.pub_cmd_vel);
    left_string_pub.publish(car_state.pub_left_string);
    right_string_pub.publish(car_state.pub_right_string);

    // circle(src, cv::Point(image_width/2,image_height*0.75), 20, Scalar(0, 0, 0), CV_FILLED, CV_AA);
    imshow("img",src);
    imshow("word",word);
    cv::waitKey(1);
}

car_ctrl_ car_ctrl(char direction,float R)
{
    car_ctrl_ car_state;

    if(keyboard.w==1)
    {
        car_state.pub_cmd_vel.linear.x=10;
    }
    else if(keyboard.s==1)
    {
        car_state.pub_cmd_vel.linear.x=-10;
    }
    else if(keyboard.w==0&&keyboard.s==0)
    {
        car_state.pub_cmd_vel.linear.x=0;
    }

    // double angle_=asin(1.6/(R/100));
    // car_state.pub_left_string.data=angle_;
    // car_state.pub_right_string.data=angle_;

    if(keyboard.a==1)
    {
        car_state.pub_left_string.data=10.0/180*CV_PI;
        car_state.pub_right_string.data=10.0/180*CV_PI;
    }
    else if(keyboard.d==1)
    {
        car_state.pub_left_string.data=-10.0/180*CV_PI;
        car_state.pub_right_string.data=-10.0/180*CV_PI;
    }
    else if(keyboard.a==0&&keyboard.d==0)
    {
        car_state.pub_left_string.data=0;
        car_state.pub_right_string.data=0;
    }

    return car_state;
}

static void sig_handler(int sig)
{
    ros::shutdown();
    cv::destroyAllWindows();
}

int main(int argc, char **argv)
{
    YAML::Node config = YAML::LoadFile("/home/wust/Personal_Data/ros_ws/robot-my_ws/src/robot/robot_program/config/camera.yaml");
    vector<double> ext=config["camera_matrix"]["data"].as<vector<double>>();
    assert((int)ext.size()==9);
    for(int row=0;row<3;row++)
    {
        for(int col=0;col<3;col++)
        {
            matrix.K.at<double>(row,col) = ext[row*3+col];
        }
    }

    vector<double> ext_=config["distortion_coefficients"]["data"].as<vector<double>>();
    assert((int)ext_.size()==5);
    for(int row=0;row<5;row++)
    {
        matrix.distCoeff.at<double>(row,0) = ext_[row];
    }

    //Initiate ROS
    ros::init(argc, argv, "subscribe_and_publish", ros::init_options::NoSigintHandler);

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish test;
    ros::MultiThreadedSpinner s(6);  //多线程
    ros::spin(s);

    return 0;
}

Mat polyfit(vector<cv::Point>& in_point, int n , char x)
{
	int size = in_point.size();
	//所求未知数个数
	int x_num = n + 1;
	//构造矩阵U和Y
	Mat mat_u(size, x_num, CV_64F);
	Mat mat_y(size, 1, CV_64F);

	for (int i = 0; i < mat_u.rows; ++i)
		for (int j = 0; j < mat_u.cols; ++j)
		{
            if(x=='x')
			    mat_u.at<double>(i, j) = pow(in_point[i].x, j);
            else if(x=='y')
                mat_u.at<double>(i, j) = pow(in_point[i].y, j);
		}

	for (int i = 0; i < mat_y.rows; ++i)
	{
        if(x=='x')
		    mat_y.at<double>(i, 0) = in_point[i].y;
        else if(x=='y')
            mat_y.at<double>(i, 0) = in_point[i].x;
	}

	//矩阵运算，获得系数矩阵K
	Mat mat_k(x_num, 1, CV_64F);
	mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
	// cout << mat_k << endl;
	return mat_k;
}

bool cmp(color_ x,color_ y)
{
	return x.point.y > y.point.y;
}

box_ box_sort(box_ box)
{
    box_ box_dst;
    static cv::Point base(image_width/2,image_height);

    sort(box.blue,box.blue+box.count.blue,cmp);
    sort(box.red,box.red+box.count.red,cmp);
    sort(box.yellow,box.yellow+box.count.yellow,cmp);

    // point_sorting(box.blue,'b');
    // point_sorting(box.red,'r');

    for(int i=0;i<box.count.blue;i++)
        box_dst.blue[i]=box.blue[i];
    for(int i=0;i<box.count.red;i++)
        box_dst.red[i]=box.red[i];
    for(int i=0;i<box.count.yellow;i++)
        box_dst.yellow[i]=box.yellow[i];

    box_dst.count=box.count;
    box_dst.sign=box.sign;
    return box_dst;
}

int point_distance(color_ color[] , char sign)
{
    vector<color_> color_vector;
    cv::Point P1;
    int count;
    switch(sign)
    {
        case 'b':
            count=box.count.blue;
            break;
        case 'r':
            count=box.count.red;
            break;
    }

    if(count<=1)
    {
        return 0;
    }

    for(int i=0;i<count;i++)
    {
        color_vector.push_back(color[i]);
    }

    for(int i=1;i<count;i++)
    {
        P1=color[0].point;
        if(color[i].point.y<P1.y)
        {
            P1=color[i].point;
        }
    }

    for(int i=0;i<count;i++)
    {
        int distance_min=image_width*image_height;
        for(int j=0 ; j<color_vector.size() ; j++)
        {
            int distance;
            cv::Point P2=color_vector[j].point;
            distance=pow(P2.x-P1.x, 2)+pow(P2.y-P1.y, 2);
            if(distance<distance_min)
            {
                distance_min=distance;
                color[i]=color_vector[j];
            }
        }

        P1=color[i].point;

        for(int k=0;k<color_vector.size();k++)
        {
            if(color_vector[k].id==color[i].id)
            {
                color_vector.erase(color_vector.begin()+k);
                break;
            }
        }
    }

    return 0;
}

//阶乘
int factorial(int n)
{
    int x=1;
    while(n>=1)
    {
        x*=n;
        n=n-1;
    }
    return x;
}

cv::Mat matrix_R(double x,double y, double z)
{
    double sinx=sin(x);
    double cosx=cos(x);
    double siny=sin(y);
    double cosy=cos(y);
    double sinz=sin(z);
    double cosz=cos(z);

    cv::Mat matrix=Mat::zeros(3,3,CV_64F);
    cv::Mat mat_x=Mat::zeros(3,3,CV_64F);
    cv::Mat mat_y=Mat::zeros(3,3,CV_64F);
    cv::Mat mat_z=Mat::zeros(3,3,CV_64F);

    mat_x=(Mat_<double>(3,3)  <<     1.0,  0.0,  0.0,
                                                                        0.0,  cosx,  sinx,
                                                                        0.0,  -sinx,  cosx);
    mat_y=(Mat_<double>(3,3)  <<     cosy,  0.0,  -siny,
                                                                        0.0,  1.0,  0.0,
                                                                        siny,  0.0,  cosy);
    mat_z=(Mat_<double>(3,3)  <<     cosz,  sinz,  0.0,
                                                                        -sinz,  cosz,  0.0,
                                                                        0.0,  0.0,  1.0);
    matrix=mat_z*mat_y*mat_x;

    return matrix;
}

cv::Point3d Towordpoint(cv::Point P2, int h)
{
    cv::Point3d P3=cv::Point3d(0,0,0);
    cv::Mat R_=Mat::zeros(3,3,CV_64F);
    cv::Mat K_=Mat::zeros(3,3,CV_64F);
    cv::Mat UV=Mat::zeros(3,1,CV_64F);
    UV=(Mat_<double>(3,1)  <<  (double)P2.x,  (double)P2.y,  1.0);
    cv::Mat mat1=Mat::zeros(3,1,CV_64F);
    cv::Mat mat2=Mat::zeros(3,1,CV_64F);
    cv::Mat word=Mat::zeros(3,1,CV_64F);
    cv::invert(matrix.R,R_);
    cv::invert(matrix.K,K_);
    mat1=R_*K_*UV;
    mat2=R_*matrix.T;

    double ZC=(h+mat2.at<double>(2,0))/mat1.at<double>(2,0);
    // word=mat1*ZC-mat2;
    // P.x=word.at<double>(0,0);
    // P.y=word.at<double>(1,0);
    // P.z=word.at<double>(2,0);

    P3.x=ZC*mat1.at<double>(0,0)-mat2.at<double>(0,0);
    P3.y=ZC*mat1.at<double>(1,0)-mat2.at<double>(1,0);
    P3.z=ZC*mat1.at<double>(2,0)-mat2.at<double>(2,0);

    //cout<<matrix.T<<endl<<K_<<endl<<endl;
    //cout<<mat1<<endl<<mat2<<endl<<endl;

    return P3;
}

cv::Point Touvpoint(int x , int y , int z)
{
    cv::Mat uv1=Mat::zeros(3,1,CV_64F);
    cv::Mat mat=Mat::zeros(3,1,CV_64F);
    cv::Mat word=(Mat_<double>(3,1)  <<  (double)x,  (double)y,  (double)z);
    mat=matrix.K*(matrix.R*word+matrix.T);
    double zc=mat.at<double>(2,0);
    uv1=mat/zc;
    cv::Point P=cv::Point((int)uv1.at<double>(0,0),(int)uv1.at<double>(1,0));

    return P;
}

////////////////////////////////////检测A点与B点是否为同一点////////////////////////////////////////
int test_point(cv::Point3d A,cv::Point3d B, double x_range, double y_range)
{
    int sign=0;
    double xmin=A.x-x_range;
    double xmax=A.x+x_range;
    double ymin=A.y-y_range;
    double ymax=A.y+y_range;

    if(B.x>xmin&&B.x<xmax&&B.y>ymin&&B.y<ymax)
    {
        sign=1;
    }

    return sign;

}


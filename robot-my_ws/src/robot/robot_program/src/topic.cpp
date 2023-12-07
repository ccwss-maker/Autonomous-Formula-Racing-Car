#include <topic.hpp>
#include <route_planning.hpp>

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
        // joint_states_sub = n_.subscribe("/car/joint_states", 1, &SubscribeAndPublish::joint_states_callback, this);
        kbd_sub = n_.subscribe("/keyboard", 1, &SubscribeAndPublish::kbd_callback, this);
        boxes_sub = n_.subscribe("/yolov5/boxes", 1, &SubscribeAndPublish::boxes_callback, this);
    }

    void image_callback(const ImageConstPtr & msg);
    void boxes_callback(const box_ros_msgs::BoundingBoxesConstPtr &boxes);
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
    Subscriber boxes_sub;
    Subscriber kbd_sub;
    //Subscriber imu_sub;

    Twist pub_cmd_vel;
    Float64 pub_left_string;
    Float64 pub_right_string;
};
void SubscribeAndPublish::boxes_callback(const box_ros_msgs::BoundingBoxesConstPtr &boxes)
{
    //////////////////////////////////////////////odom//////////////////////////////////////////
    tf::Quaternion quat;
    tf::quaternionMsgToTF(boxes->odom.pose.pose.orientation,quat);
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
    double x=boxes->odom.pose.pose.position.x;
    double y=boxes->odom.pose.pose.position.y;
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

        cv::Point3d point_word;
        // int area=(xmax-xmin)*(ymax-ymin);
        // if(area<180)
        // {
        //     continue;
        // }

        cv::Point3d point_1,point_2;
        point_1 = Towordpoint(cv::Point(xmax,ymax),0);
        point_2 = Towordpoint(cv::Point(xmin,ymax),0);
        point_word.x=(point_1.x+point_2.x)/2;
        point_word.y=(point_1.y+point_2.y)/2;

        if(boxes->bounding_boxes[i].Class[0]=='r')
        {
            box.red[count_r].name='r';
            box.red[count_r].id=count_r;
            box.red[count_r].point.x=(xmax+xmin)/2;
            box.red[count_r].point.y=(ymax+ymin)/2;
            box.red[count_r].Point3=point_word;
            box.red[count_r].point_tl=cv::Point(xmin,ymin);
            box.red[count_r].point_tr=cv::Point(xmax,ymin);
            box.red[count_r].point_bl=cv::Point(xmin,ymax);
            box.red[count_r].point_br=cv::Point(xmax,ymax);
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
            box.blue[count_b].point_tl=cv::Point(xmin,ymin);
            box.blue[count_b].point_tr=cv::Point(xmax,ymin);
            box.blue[count_b].point_bl=cv::Point(xmin,ymax);
            box.blue[count_b].point_br=cv::Point(xmax,ymax);
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
            box.yellow[count_y].point_tl=cv::Point(xmin,ymin);
            box.yellow[count_y].point_tr=cv::Point(xmax,ymin);
            box.yellow[count_y].point_bl=cv::Point(xmin,ymax);
            box.yellow[count_y].point_br=cv::Point(xmax,ymax);
            count_y++;
        }
    }
    // cout<<endl;
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
    Mat word=Mat::zeros(boxes->img.height,boxes->img.width,CV_8UC3);
    Mat camera=Mat::zeros(boxes->img.height,boxes->img.width,CV_8UC3);
    double x_convert=1.0*boxes->img.width/160000;
    double y_convert=1.0*boxes->img.height/100000;
    vector<cv::Point> blue_point;
    vector<cv::Point> red_point;
    vector<cv::Point> road_point;
    static vector<Point2d_> blue_point_w;
    static vector<Point2d_> red_point_w;
    static vector<Point2d_> yellow_point_w;
    vector<cv::Point2d> blue_point_c;
    vector<cv::Point2d> red_point_c;
    vector<cv::Point2d> yellow_point_c;
    if(init_sign==0)
    {
        image_width=boxes->img.width;
        image_height=boxes->img.height;
        namedWindow("img",WINDOW_FREERATIO);
        namedWindow("word",WINDOW_FREERATIO);
        namedWindow("camera",WINDOW_FREERATIO);
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
    // mono8: CV_8UC1, grayscale image
    // mono16: CV_16UC1, 16-bit grayscale image
    // bgr8: CV_8UC3, color image with blue-green-red color order
    // rgb8: CV_8UC3, color image with red-green-blue color order
    // bgra8: CV_8UC4, BGR color image with an alpha channel
    // rgba8: CV_8UC4, RGB color image with an alpha channel
    cv_ptr = toCvCopy(source, "bgr8");
    src=cv_ptr->image;
    
    /////////////////////////////////////////////画出车的坐标////////////////////////////////////////////////
    double car_p_x=-boxes->odom.pose.pose.position.x*1000;
    double car_p_y=boxes->odom.pose.pose.position.y*1000;
    cv::Point2d car_p_3(car_p_x,car_p_y);
    cv::Point car_p = convert(car_p_3,boxes->img.width,boxes->img.height,x_convert,y_convert);
    circle(word,car_p,5,Scalar(255,255,255),CV_FILLED,CV_AA);
    circle(camera,cv::Point(boxes->img.width/2,boxes->img.height/2),5,Scalar(255,255,255),CV_FILLED,CV_AA);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    YAML::Node point_node = YAML::LoadFile("/home/wust/Personal_Data/ros_ws/robot-my_ws/src/robot/robot_program/config/point.yaml");
    int distance_limt=point_node["point"]["distance"].as<std::int64_t>();
    for(int i =0;i<box.count.red;i++)
    {
        roadblock_box(&src,box.red[i],Scalar(0,0,255));
        // circle(src,box.red[i].point,10,Scalar(255,0,0),CV_FILLED,CV_AA);
        Point2d_ P = ToPoint2d(box.red[i] , car_p_3);
        roadblock_point_w(&red_point_w,P,distance_limt);
    }

    for(int i=0;i<box.count.blue;i++)
    {
        roadblock_box(&src,box.blue[i],Scalar(255,0,0));
        // circle(src,box.blue[i].point,10,Scalar(0,0,255),CV_FILLED,CV_AA);
        Point2d_ P = ToPoint2d(box.blue[i] , car_p_3);
        roadblock_point_w(&blue_point_w,P,distance_limt);
    }

    for(int i=0;i<box.count.yellow;i++)
    {
        roadblock_box(&src,box.yellow[i],Scalar(0,255,255));
        // circle(src,box.yellow[i].point,10,Scalar(255,255,255),CV_FILLED,CV_AA);
        Point2d_ P = ToPoint2d(box.yellow[i] , car_p_3);
        roadblock_point_w(&yellow_point_w,P,distance_limt);
    }

    for(int i=0;i<red_point_w.size();i++)
    {
        cv::Point P=convert(red_point_w[i].point2d,boxes->img.width,boxes->img.height,x_convert,y_convert);
        circle(word,P,3,Scalar(0,0,255),CV_FILLED,CV_AA);
    }

    for(int i=0;i<blue_point_w.size();i++)
    {
        cv::Point P=convert(blue_point_w[i].point2d,boxes->img.width,boxes->img.height,x_convert,y_convert);
        circle(word,P,3,Scalar(255,0,0),CV_FILLED,CV_AA);
    }

    for(int i=0;i<yellow_point_w.size();i++)
    {
        cv::Point P=convert(yellow_point_w[i].point2d,boxes->img.width,boxes->img.height,x_convert,y_convert);
        circle(word,P,3,Scalar(0,255,255),CV_FILLED,CV_AA);
    }
///////////////////////////////////////////////将世界坐标系转为相机坐标系////////////////////////////////////////////////////////
    roadblock_point_c(&red_point_c,red_point_w,1000*x,1000*y,-pitch);
    roadblock_point_c(&blue_point_c,blue_point_w,1000*x,1000*y,-pitch);
    roadblock_point_c(&yellow_point_c,yellow_point_w,1000*x,1000*y,-pitch);

/////////////////////////////////////////////////////创建目标点////////////////////////////////////////////////////////////////////////
    vector<cv::Point2d> red_point_c_;   
    vector<cv::Point2d> blue_point_c_;
    for(int i=0;i<red_point_c.size();i++)
    {
        if(red_point_c[i].y>0&&red_point_c[i].y<20000&&red_point_c[i].x>-5000&&red_point_c[i].x<5000)
        {
            red_point_c_.push_back(red_point_c[i]);
            cv::Point P=convert(red_point_c[i],boxes->img.width,boxes->img.height,x_convert,y_convert);
            circle(camera,P,3,Scalar(0,0,255),CV_FILLED,CV_AA);
        }
    }

    for(int i=0;i<blue_point_c.size();i++)
    {
        if(blue_point_c[i].y>0&&blue_point_c[i].y<20000&&blue_point_c[i].x>-5000&&blue_point_c[i].x<5000)
        {
            blue_point_c_.push_back(blue_point_c[i]);
            cv::Point P=convert(blue_point_c[i],boxes->img.width,boxes->img.height,x_convert,y_convert);
            circle(camera,P,3,Scalar(255,0,0),CV_FILLED,CV_AA);
        }
    }

    for(int i=0;i<yellow_point_c.size();i++)
    {
        if(yellow_point_c[i].y>0&&yellow_point_c[i].y<20000&&yellow_point_c[i].x>-5000&&yellow_point_c[i].x<5000)
        {
            cv::Point P=convert(yellow_point_c[i],boxes->img.width,boxes->img.height,x_convert,y_convert);
            circle(camera,P,3,Scalar(0,255,255),CV_FILLED,CV_AA);
        }
    }
    vector<cv::Point2d> P_car_target = car_target(red_point_c_,blue_point_c_);
    for(int i=0 ; i<P_car_target.size();i++)
    {
        cv::Point P=convert(P_car_target[i],boxes->img.width,boxes->img.height,x_convert,y_convert);
        circle(camera,P,3,Scalar(255,255,255),CV_FILLED,CV_AA);
        cout<<P_car_target[i]<<endl;
    }
    cout<<endl;
    //////////////////////////////////////////////////////////////////////////////////////
    int node = point_node["control"].as<std::int64_t>();
    if(node == 1)
    {
        angle_ angle_wheel;
        CarState_ carstate;
        car_ctrl_ car_state;
        carstate.car_L = 2.4;
        carstate.car_W = 1;
        double v_x = boxes->odom.twist.twist.linear.x;
        double v_y = boxes->odom.twist.twist.linear.y;
        double v_z = boxes->odom.twist.twist.linear.z;

        double kv = point_node["puresuit"]["kv"].as<std::double_t>();
        double d0 = point_node["puresuit"]["d0"].as<std::double_t>();

        carstate.speed = pow(pow(v_x , 2) + pow(v_y , 2) + pow(v_z , 2) , 0.5);
        angle_wheel = followPurePursuit(carstate , P_car_target , kv , d0);

        angle_wheel.p_fit.x*=1000;
        angle_wheel.p_fit.y*=1000;
        circle(camera,angle_wheel.p_fit,2,Scalar(0,255,255),CV_FILLED,CV_AA);

        cout<<"fit"<<angle_wheel.p_fit<<endl;
        car_state.pub_cmd_vel.linear.x = point_node["car_v"].as<std::double_t>();
        car_state.pub_left_string.data = angle_wheel.angle_L;
        car_state.pub_right_string.data = angle_wheel.angle_R;
        
        cmd_vel_pub.publish(car_state.pub_cmd_vel);
        left_string_pub.publish(car_state.pub_left_string);
        right_string_pub.publish(car_state.pub_right_string);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    cv::imshow("img",src);
    cv::imshow("word",word);
    cv::imshow("camera",camera);
    cv::waitKey(1);
}

void SubscribeAndPublish::kbd_callback(const kbd_ros_msgs::kbdConstPtr & msg)
{
    YAML::Node node = YAML::LoadFile("/home/wust/Personal_Data/ros_ws/robot-my_ws/src/robot/robot_program/config/point.yaml");
    int node_ = node["control"].as<std::int64_t>();
    if(node_ == 0)
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

/////////////////////////////////像素坐标系转换世界坐标系//////////////////////////////////////
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
    P3.x=ZC*mat1.at<double>(0,0)-mat2.at<double>(0,0);
    P3.y=ZC*mat1.at<double>(1,0)-mat2.at<double>(1,0);
    P3.z=ZC*mat1.at<double>(2,0)-mat2.at<double>(2,0);

    
    // word=mat1*ZC-mat2;
    // P.x=word.at<double>(0,0);
    // P.y=word.at<double>(1,0);
    // P.z=word.at<double>(2,0);

    

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
int test_point(Point2d_ A,Point2d_ B, double x_range, double y_range)
{
    int sign=0;
    double xmin=A.point2d.x-x_range;
    double xmax=A.point2d.x+x_range;
    double ymin=A.point2d.y-y_range;
    double ymax=A.point2d.y+y_range;

    if(B.point2d.x>xmin&&B.point2d.x<xmax&&B.point2d.y>ymin&&B.point2d.y<ymax)//B点在A点的范围内
    {
        sign=1;
    }

    return sign;

}
///////////////////////////////////////////坐标转换 p_word->p_uv///////////////////////////////////////////////////
cv::Point convert(cv::Point2d p_uv,int width,int height,double x_convert,double y_convert)
{
    cv::Point p_word;
    p_word.x = width/2-x_convert*p_uv.x;
    p_word.y = height/2-y_convert*p_uv.y;
    return p_word;
}
///////////////////////////////////////////坐标转换 p_word->p_c///////////////////////////////////////////////////
void roadblock_point_c(vector<Point2d> *point_c , vector<Point2d_> point_w , double x , double y , double angle)
{
    for(int i=0 ; i < point_w.size() ; i++)
    {
        cv::Point2d P_T,P_C;
        P_T.x = -point_w[i].point2d.x-x;
        P_T.y = point_w[i].point2d.y-y;
        P_C.x = -(P_T.x*cos(angle)+P_T.y*sin(angle));
        P_C.y = -P_T.x*sin(angle)+P_T.y*cos(angle);
        point_c->push_back(P_C);
    }
}
//////////////////////////////////////////计算锥桶与车的距离///////////////////////////////////////////////////
Point2d_ ToPoint2d(color_ box , cv::Point2d car)
{
    Point2d_ point2d;
    point2d.point2d.x=box.Point3.x;
    point2d.point2d.y=box.Point3.y;
    point2d.distance=pow(pow((car.y-box.Point3.y)/1000,2)+pow((car.x-box.Point3.x)/1000,2),0.5);
    return point2d;
}

//////////////////////////////////////////过滤相同位置锥桶///////////////////////////////////////////////////
void roadblock_point_w(vector<Point2d_> *point_w , Point2d_ P , double distance_limt)
{
    if(P.distance <= distance_limt)
    {
        int sign_find = 0; /////找相同点 ////0：无 ; 1：有

        for(int j=0;j<point_w->size();j++)
        {
            if(test_point(point_w->at(j), P))
            {
                cv::Point2d point_avg;
                point_avg.x=(point_w->at(j).point2d.x+P.point2d.x)/2;
                point_avg.y=(point_w->at(j).point2d.y+P.point2d.y)/2;
                point_w->at(j).point2d = point_avg;
                
                sign_find = 1;
                break;
            }
        }
        if(!sign_find)
        {
            point_w->push_back(P);
        }
    }
}
////////////////////////////框出锥桶///////////////////////////////////////
void roadblock_box(Mat *src,color_ color,const cv::Scalar &scalar)
{
    cv::line(*src,color.point_tl,color.point_tr,scalar);
    cv::line(*src,color.point_tl,color.point_bl,scalar);
    cv::line(*src,color.point_br,color.point_bl,scalar);
    cv::line(*src,color.point_br,color.point_tr,scalar);
}
/////////////////////////创建目标点//////////////////////////////////
vector<cv::Point2d> car_target(vector<cv::Point2d> red_point_c , vector<cv::Point2d> blue_point_c)
{
    vector<cv::Point2d> P;
    int num = red_point_c.size() > blue_point_c.size() ? blue_point_c.size() : red_point_c.size();
    cv::Point2d point[10];
    for(int i=0;i<num;i++)
    {
        point[i].x=(red_point_c[i].x+blue_point_c[i].x)/2;
        point[i].y=(red_point_c[i].y+blue_point_c[i].y)/2;
    }
    point_sort(point,10);
    for(int i=0;i<num;i++)
    {
        P.push_back(point[i]);
    }
    return P;
}
////////////////////////////////////按照Y坐标排序//////////////////////////////////////////
bool cmp(cv::Point2d p1,cv::Point2d p2)
{
	return p1.y < p1.y;
}

void point_sort(cv::Point2d point[], int num)
{
    sort(point,point+num,cmp);
}
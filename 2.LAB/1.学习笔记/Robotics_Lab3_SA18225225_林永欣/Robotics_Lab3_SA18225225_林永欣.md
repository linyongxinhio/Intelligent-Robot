# 实验 3 图像特征匹配、跟踪与相机运动估计实验

## 一.实验代码分析

### <1> 双目图像特征匹配

#### 1.完整代码

```C++
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main ( int argc, char** argv )
{
    if ( argc != 3 )
    {
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }
    
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    //-- 初始化
    std::vector<KeyPoint> keypoints_1, keypoints_2;  //关键点
    Mat descriptors_1, descriptors_2;  //描述子
    Ptr<FeatureDetector> detector = ORB::create();  
    Ptr<DescriptorExtractor> descriptor = ORB::create();  
    Ptr<DescriptorMatcher> matcher=DescriptorMatcher::create ( "BruteForce-Hamming" );

    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    Mat outimg1;
    drawKeypoints(img_1,keypoints_1, outimg1, Scalar::all(-1), 		                                     DrawMatchesFlags::DEFAULT );
    imshow("ORB特征点",outimg1);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用汉明距离
    vector<DMatch> matches;
    matcher->match ( descriptors_1, descriptors_2, matches );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
    // 仅供娱乐的写法
    min_dist = min_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;
    max_dist = max_element( matches.begin(), matches.end(), [](const DMatch& m1, const DMatch& m2) {return m1.distance<m2.distance;} )->distance;

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //优化匹配，当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }

    //-- 第五步:绘制匹配结果
    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
    drawMatches ( img_1, keypoints_1, img_2,keypoints_2, good_matches, img_goodmatch );
    imshow ( "所有匹配点对", img_match );
    imshow ( "优化后匹配点对", img_goodmatch );
    waitKey(0);

    return 0;
}

```



#### 2.代码分析

下面从代码细节来一步步分析。

```c++
int main ( int argc, char** argv )
{
    if ( argc != 3 )
    {
        cout<<"usage: feature_extraction img1 img2"<<endl;
        return 1;
    }
}
```

> **main函数的两个参数详解**
>
> 这里argc存储了传入程序的参数个数，由于第一个参数默认是程序本身，又传入了两张图片，因此参数个数为3个。
>
> **知识点总结：**
>
> argc是命令行总的参数个数,argv[]是argc个参数，其中第0个参数是程序的全名，后面的参数对应用户输入的参数。比如在命令行输入：test   a.c   b.c   t.c，第0个参数是test，第1个参数是a.c，第2个参数是b.c，第3个参数是t.c。
>
> char *argv[]是一个字符数组,其大小是int argc，主要用于命令行参数argv[]参数，数组里每个元素代表一个参数。
>
> 一般我们写程序main函数都是不带参数的，也就是main 后的括号都是空括号。实际上，main函数可以带参数，这个参数可以认为是 main函数的形式参数。Ｃ/C++规定main函数的参数只能有两个， 习惯上这两个参数写为argc和argv。因此，main函数的函数头可写为： main (argc,argv)。
>
> 由于main函数不能被其它函数调用， 因此不可能在程序内部取得实际值。那么，在何处把实参值赋予main函数的形参呢? 实际上,main函数的参数值是从操作系统命令行上获得的。当我们要运行一个可执行文件时，在DOS提示符下键入文件名，再输入实际参数即可把这些实参传送到main的形参中去。
>
> DOS提示符下命令行的一般形式为： C:\>可执行文件名 参数 参数……; 但是应该特别注意的是，main 的两个形参和命令行中的参数在位置上不是一一对应的。因为,main的形参只有二个，而命令行中的参数个数原则上未加限制。argc参数表示了命令行中参数的个数(注意：文件名本身也算一个参数)，argc的值是在输入命令行时由系统按实际参数的个数自动赋予的。
>
> **参考链接：**https://blog.csdn.net/hopeneversleep/article/details/55798722

```c++
std::vector<KeyPoint> keypoints_1, keypoints_2;  //关键点
```

> 首先定义两个数据类型为KeyPoint的vector容器，名字分别为keypoints_1和keypoints_2。这两个容器是准备存放两张图像所提取出的特征点的。那么对于特征点，我们知道最起码需要确定他在图像中的位置，进而如果需要使用改进BRIEF去计算其描述子，我们还需知道其方向，等等。
>
> KeyPoint类型的参数列表如下：
>
> - angle：角度，表示关键点的方向。为了保证方向不变形，SIFT算法通过对关键点周围邻域进行梯度运算，求得该点方向。
> - class_id：当要对图片进行分类时，我们可以用class_id对每个特征点进行区分，未设定时为-1，需要靠自己设定。
> - octave：代表是从金字塔哪一层提取的得到的数据。
> - pt：关键点的坐标。
> - response：响应程度，代表该点强壮大小。
> - size：该点直径的大小。
>
> 之后我们便可以调用函数来提取每个图像中的特征点信息，并存放于两个vcetor容器中。
>
> **参考链接：**https://www.cnblogs.com/my-idiot-days/archive/2013/05/01/3053831.html

```c++
Ptr<FeatureDetector> detector = ORB::create();  
```

> **特征检测器FeatureDetector **
>
> FeatureDetetor是虚类，通过定义FeatureDetector的对象可以使用多种特征检测方法。通过create()函数调用。
>
> ```
> Ptr<FeatureDetector> FeatureDetector::create(const string& detectorType);
> ```
>
> **参考链接：**https://tw.saowen.com/a/196dab63fcf38b1880781f9c1ab87b53af83588146d9c907b2e3b5cd198e9c69

```c++
Ptr<DescriptorExtractor> descriptor = ORB::create();  
```

> **特征描述子提取公用接口DescriptorExtractor**
>
> DescriptorExtractor是特征描述子提取公用接口，所有实现 `vector` 特征描述子子提取的部分继承了 DescriptorExtractor接口。
>
> ```c++
> Ptr<DescriptorExtractor> DescriptorExtractor::create(const string& descriptorExtractorType)
> ```
>
> **参考链接：**http://www.opencv.org.cn/opencvdoc/2.3.2/html/modules/features2d/doc/common_interfaces_of_descriptor_extractors.html                              

```c++
Ptr<DescriptorMatcher> matcher=DescriptorMatcher::create ( "BruteForce-Hamming" );
```

> **特征描述子匹配公用接口DescriptorMatcher  **
>
> DescriptorMatcher是用于特征关键点描述子匹配的抽象基类。 有两类匹配任务：匹配两个图像之间的特征描述子，或者匹配一个图像与另外一个图像集的特征描述子。这里使用的是**汉明距离**。
>
> **参考链接:**http://www.opencv.org.cn/opencvdoc/2.3.2/html/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html?highlight=descriptormatcher#DescriptorMatcher

```c++
// 对两幅图像中的BRIEF描述子进行匹配，使用汉明距离
vector<DMatch> matches;
matcher->match ( descriptors_1, descriptors_2, matches );
```

> **DescriptorMatcher中的match用法**
>
> 给定查询集合中的每个特征描述子，寻找最佳匹配。
>
> 参数列表如下：
>
> - **queryDescriptors** – 特征描述子查询集.
> - **trainDescriptors** – 待训练的特征描述子集. 这个集没被加载到类的对象中.
> - **matches** – Matches. `matches` 尺寸小于超汛特征描述子的数量.
> - **mask** – 特定的在输入查询和训练特征描述子集之间的Mask permissible匹配.
> - **masks** – masks集. 每个 `masks[i]` 特定标记出了在输入查询特征描述子和存储的从第i个图像中提取的特征描述子集
>
> **参考链接：**http://www.opencv.org.cn/opencvdoc/2.3.2/html/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html?highlight=descriptormatcher#DescriptorMatcher

```c++
drawKeypoints(img_1,keypoints_1, outimg1, Scalar::all(-1),DrawMatchesFlags::DEFAULT );
```

> drawKeypoints函数的参数列表如下：
>
> - **image** – 源图像。
> - **keypoints** – 源图像中的关键点。
> - **outImg** –  输出图像。其内容取决于定义输出图像中绘制内容的标志值。
> - **color** – 关键点的颜色。通过修改（b,g,r）的值,更改画笔的颜色，b=蓝色，g=绿色，r=红色。
> - **flags** – 设置绘图功能的标志。
>
> 这里首先定义了一个Mat类变量outimg1，顾名思义是要将特征点在img1上画出来并存为一个新的图像来进行展示。调用了drawKeypoints函数使用默认值将img_1与keypoints_1中存储的特征点进行圈画。
>
> **参考链接：**https://www.codetd.com/article/167763

```c++
std::vector< DMatch > good_matches;
```

> **Dmatch类**
>
> 这里又定义了一个容器，存储的对象类型为cv::DMatch。DMatch类型的变量用来存储特征点之间的匹配情况，每个变量有四个成员对象：queryIdx、trainIdx、imgIdx、distance。在不考虑图像下标imgIdx时，queryIdx与trainIdx构成了一组匹配好的特征点分别在两张图像内的索引，分别是所选中的特征点在所对应的keypoints容器中对应的标号。Distance代表配好对的特征点之间的距离，这里使用汉明距离，距离越小越好。进而，使用matcher中的match函数，将存有描述子信息的Mat类变量descriptors_1与descriptors_2进行相似度匹配，并存于matches容器中。
>
>  **参考链接：**https://blog.csdn.net/Quincuntial/article/details/50114773

```c++
Mat img_match;
drawMatches ( img_1, keypoints_1, img_2, keypoints_2, matches, img_match );
```

> 这里调用drawMatches函数对两张图像img_1、img_2以及其之间的特征点配对进行连线与拼接，将左右两张图拼接成一张图并存入Mat类型对象img_match中。

#### 3.输出结果

![](C:\Users\linyongxin\AppData\Roaming\Typora\typora-user-images\1546344281861.png)

​								图1 img_1与该帧图像中提取的FAST特征点

![1546344602881](C:\Users\linyongxin\AppData\Roaming\Typora\typora-user-images\1546344602881.png)

​								图2 使用暴力匹配后的特征点配对情况

![1546344632303](C:\Users\linyongxin\AppData\Roaming\Typora\typora-user-images\1546344632303.png)

​						图3 对特征点对距离distance进行筛选后的优化配对结果

---

### <2> ICP 法相机姿态估计

#### 1.完整代码

```c++
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

using namespace std;
using namespace cv;

void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

// 像素坐标转相机归一化坐标
Point2d pixel2cam ( const Point2d& p, const Mat& K );

void pose_estimation_3d3d (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t
);

void bundleAdjustment(
    const vector<Point3f>& points_3d,
    const vector<Point3f>& points_2d,
    Mat& R, Mat& t
);

// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly( const Eigen::Vector3d& point ) : _point(point) {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // measurement is p, point is p'
        _error = _measurement - pose->estimate().map( _point );
    }

    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -z;
        _jacobianOplusXi(0,2) = y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;

        _jacobianOplusXi(1,0) = z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = -x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;

        _jacobianOplusXi(2,0) = -y;
        _jacobianOplusXi(2,1) = x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }

    bool read ( istream& in ) {}
    bool write ( ostream& out ) const {}
protected:
    Eigen::Vector3d _point;
};

int main ( int argc, char** argv )
{
    if ( argc != 5 )
    {
        cout<<"usage: pose_estimation_3d3d img1 img2 depth1 depth2"<<endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread ( argv[1], CV_LOAD_IMAGE_COLOR );
    Mat img_2 = imread ( argv[2], CV_LOAD_IMAGE_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    // 建立3D点
    // 深度图为16位无符号数，单通道图像
    Mat depth1 = imread ( argv[3], CV_LOAD_IMAGE_UNCHANGED );
    // 深度图为16位无符号数，单通道图像
    Mat depth2 = imread ( argv[4], CV_LOAD_IMAGE_UNCHANGED );
    
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point3f> pts1, pts2;

    for ( DMatch m:matches )
    {
        ushort d1 = depth1.ptr<unsigned short> ( int ( keypoints_1[m.queryIdx].pt.y ) ) [ int ( keypoints_1[m.queryIdx].pt.x ) ];
        ushort d2 = depth2.ptr<unsigned short> ( int ( keypoints_2[m.trainIdx].pt.y ) ) [ int ( keypoints_2[m.trainIdx].pt.x ) ];
        if ( d1==0 || d2==0 )   // bad depth
            continue;
        Point2d p1 = pixel2cam ( keypoints_1[m.queryIdx].pt, K );
        Point2d p2 = pixel2cam ( keypoints_2[m.trainIdx].pt, K );
        float dd1 = float ( d1 ) /5000.0;
        float dd2 = float ( d2 ) /5000.0;
        pts1.push_back ( Point3f ( p1.x*dd1, p1.y*dd1, dd1 ) );
        pts2.push_back ( Point3f ( p2.x*dd2, p2.y*dd2, dd2 ) );
    }

    cout<<"3d-3d pairs: "<<pts1.size() <<endl;
    Mat R, t;
    pose_estimation_3d3d ( pts1, pts2, R, t );
    cout<<"ICP via SVD results: "<<endl;
    cout<<"R = "<<R<<endl;
    cout<<"t = "<<t<<endl;
    cout<<"R_inv = "<<R.t() <<endl;
    cout<<"t_inv = "<<-R.t() *t<<endl;

    cout<<"calling bundle adjustment"<<endl;

    bundleAdjustment( pts1, pts2, R, t );

    // verify p1 = R*p2 + t
    for ( int i=0; i<5; i++ )
    {
        cout<<"p1 = "<<pts1[i]<<endl;
        cout<<"p2 = "<<pts2[i]<<endl;
        cout<<"(R*p2+t) = "<<
            R * (Mat_<double>(3,1)<<pts2[i].x, pts2[i].y, pts2[i].z) + t
            <<endl;
        cout<<endl;
    }
}

void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
   // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}

void pose_estimation_3d3d (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t
)
{
    Point3f p1, p2;     // center of mass
    int N = pts1.size();
    for ( int i=0; i<N; i++ )
    {
        p1 += pts1[i];
        p2 += pts2[i];
    }
    p1 = Point3f( Vec3f(p1) /  N);
    p2 = Point3f( Vec3f(p2) / N);
    vector<Point3f>     q1 ( N ), q2 ( N ); // remove the center
    for ( int i=0; i<N; i++ )
    {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for ( int i=0; i<N; i++ )
    {
        W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
    }
    cout<<"W="<<W<<endl;

    // SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd ( W, Eigen::ComputeFullU|Eigen::ComputeFullV );
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout<<"U="<<U<<endl;
    cout<<"V="<<V<<endl;

    Eigen::Matrix3d R_ = U* ( V.transpose() );
    Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) - R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );

    // convert to cv::Mat
    R = ( Mat_<double> ( 3,3 ) <<
          R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
          R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
          R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
        );
    t = ( Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) );
}

void bundleAdjustment (
    const vector< Point3f >& pts1,
    const vector< Point3f >& pts2,
    Mat& R, Mat& t )
{
	// 初始化g2o
    // pose维度为6, landmark维度为3
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverEigen<Block::PoseMatrixType>());
    // 矩阵块求解器
    std::unique_ptr<Block> solver_ptr ( new Block ( std::move(linearSolver)));
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton ( std::move(solver_ptr));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    pose->setId(0);
    pose->setEstimate( g2o::SE3Quat(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d( 0,0,0 )
    ) );
    optimizer.addVertex( pose );

    // edges
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        EdgeProjectXYZRGBDPoseOnly* edge = new EdgeProjectXYZRGBDPoseOnly(
            Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z) );
        edge->setId( index );
        edge->setVertex( 0, dynamic_cast<g2o::VertexSE3Expmap*> (pose) );
        edge->setMeasurement( Eigen::Vector3d(
            pts1[i].x, pts1[i].y, pts1[i].z) );
        edge->setInformation( Eigen::Matrix3d::Identity()*1e4 );
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose( true );
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = 
                             chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"optimization costs time: "<<time_used.count()<<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;

}

```

#### 2.代码分析

##### 1).SVD方法分析

**数学推导过程：**

![1545715177536](C:\Users\linyongxin\AppData\Roaming\Typora\typora-user-images\1545715177536.png)

**对应代码分析：**

> 1.求两个质心
>
> ```c++
> Point3f p1, p2;     // 质心
> int N = pts1.size();  
> for ( int i=0; i<N; i++ )
> {
>     p1 += pts1[i];
>     p2 += pts2[i];
> }
> p1 = Point3f( Vec3f(p1) /  N);
> p2 = Point3f( Vec3f(p2) / N);
> ```
>
> 2.定义qi和qi'，qi=pi-p，qi'=pi'-p'
>
> ```c++
> vector<Point3f>     q1 ( N ), q2 ( N );
> for ( int i=0; i<N; i++ )
> {
>     q1[i] = pts1[i] - p1;
>     q2[i] = pts2[i] - p2;
> }
> ```
>
> 3.计算W,先求qi*qi^T
>
> ```c++
> // 计算qi*qi'^T
> Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
> for ( int i=0; i<N; i++ )
> {
>     W += Eigen::Vector3d ( q1[i].x, q1[i].y, q1[i].z ) 
>          * Eigen::Vector3d ( q2[i].x, q2[i].y, q2[i].z ).transpose();
> }
> ```
>
> 4.采用SVD方法来求解U,V
>
> ```c++
> // 用SVD来求解U,V
> Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV );
> Eigen::Matrix3d U = svd.matrixU();
> Eigen::Matrix3d V = svd.matrixV();
> cout<<"U="<<U<<endl;
> cout<<"V="<<V<<endl;
> ```
>
> 5.当W满秩时，R=UV^T,t=P-RP'
>
> ```c++
> Eigen::Matrix3d R_ = U* ( V.transpose() );
> Eigen::Vector3d t_ = Eigen::Vector3d ( p1.x, p1.y, p1.z ) -
>                      R_ * Eigen::Vector3d ( p2.x, p2.y, p2.z );
> ```
>
> 6.将R,t转换成矩阵的形式
>
> ```c++
> // 讲R,t转换成矩阵的形式
> R = ( Mat_<double> ( 3,3 ) <<
>       R_ ( 0,0 ), R_ ( 0,1 ), R_ ( 0,2 ),
>       R_ ( 1,0 ), R_ ( 1,1 ), R_ ( 1,2 ),
>       R_ ( 2,0 ), R_ ( 2,1 ), R_ ( 2,2 )
>     );
> t = ( Mat_<double> ( 3,1 ) << t_ ( 0,0 ), t_ ( 1,0 ), t_ ( 2,0 ) )
> ```

##### 2).非线性优化方法分析

采用非线性优化来计算 ICP。我们依然使用李代数来表达相机位姿。与SVD 思路不同的地方在于，在优化中我们不仅考虑相机的位姿，同时会优化 3D 点的空间位置。

---

### <3> 光流特征跟踪

#### 1.完整代码

```c++
#include <iostream>
#include <fstream>
#include <list>
#include <vector>
#include <chrono>
using namespace std; 

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

int main( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: useLK path_to_dataset"<<endl;
        return 1;
    }
    string path_to_dataset = argv[1];
    string associate_file = path_to_dataset + "/associate.txt";
    
    ifstream fin( associate_file );
    if ( !fin ) 
    {
        cerr<<"I cann't find associate.txt!"<<endl;
        return 1;
    }
    
    string rgb_file, depth_file, time_rgb, time_depth;
    list< cv::Point2f > keypoints;      // 因为要删除跟踪失败的点，使用list
    cv::Mat color, depth, last_color;
    
    for ( int index=0; index<100; index++ )
    {
        fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
        color = cv::imread( path_to_dataset+"/"+rgb_file );
        depth = cv::imread( path_to_dataset+"/"+depth_file, -1 );
        if (index ==0 )
        {
            // 对第一帧提取FAST特征点
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect( color, kps );
            for ( auto kp:kps )
                keypoints.push_back( kp.pt );
            last_color = color;
            continue;
        }
        if ( color.data==nullptr || depth.data==nullptr )
            continue;
        // 对其他帧用LK跟踪特征点
        vector<cv::Point2f> next_keypoints; 
        vector<cv::Point2f> prev_keypoints;
        for ( auto kp:keypoints )
            prev_keypoints.push_back(kp);
        vector<unsigned char> status;
        vector<float> error; 
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        cv::calcOpticalFlowPyrLK( last_color, color, prev_keypoints, next_keypoints, status, error );
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
        cout<<"LK Flow use time："<<time_used.count()<<" seconds."<<endl;
        // 把跟丢的点删掉
        int i=0; 
        for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
        {
            if ( status[i] == 0 )
            {
                iter = keypoints.erase(iter);
                continue;
            }
            *iter = next_keypoints[i];
            iter++;
        }
        cout<<"tracked keypoints: "<<keypoints.size()<<endl;
        if (keypoints.size() == 0)
        {
            cout<<"all keypoints are lost."<<endl;
            break; 
        }
        // 画出 keypoints
        cv::Mat img_show = color.clone();
        for ( auto kp:keypoints )
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
        cv::imshow("corners", img_show);
        cv::waitKey(0);
        last_color = color;
    }
    return 0;
}
```

#### 2.代码分析

```c++
if ( argc != 2 )
{
    cout<<"usage: useLK path_to_dataset"<<endl;
    return 1;
}

string path_to_dataset = argv[1];
string associate_file = path_to_dataset + "/associate.txt";
```

> 首先判断argc参数，这里我们只需传入“数据集排序文件”associate.txt所在的文件夹就可以，所以argc的判别值为2。
>
> 然后定义两个string类变量，分别存储associate.txt所在文件夹的绝对路径，与associate.txt的结对路径。

```c++
ifstream fin( associate_file );
if ( !fin ) 
{
    cerr<<"I cann't find associate.txt!"<<endl;
    return 1;
}
```

> 这里实例化了一个ifstream输入文件流类的变量fin，并直接初始化为associate_file所存储的字符串。
>
> 接着判断是否能够打开fin所存储路径下的文件，这里的判断语句“！fin”并不是判断fin是否为0或者为空，而是ifstream类重载了“!”操作符，所以当我们如此使用的时候，是“!”操作符函数返回一个bool类变量来标记是否成功，成功则为1。

```c++
list< cv::Point2f > keypoints;      
```

> 使用list链表是为了方便插入与删除某个元素，这里是为了方便在后续光流法跟踪时删除跟丢的点。

```c++
for ( int index=0; index<100; index++ )
   {
      ...
   }
```

> 循环100次

```c++
fin>>time_rgb>>rgb_file>>time_depth>>depth_file;
```

> 在每次循环中，输入流fin输入associate.txt每行的数据，因为associate文件的每一行分别是time_color、color、time_depth、depth，所以分别将其赋值给存储文件名称或文件产生时间的变量。

```c++
 if (index ==0 )
 {
     // 对第一帧提取FAST特征点
     vector<cv::KeyPoint> kps;
     cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
     detector->detect( color, kps );
     for ( auto kp:kps )
         keypoints.push_back( kp.pt );
     last_color = color;
     continue;
 }
```

> 当index==0时对第一张图像进行特征点提取，按照FAST角点寻找特征点并存入keypoints中，然后在后续帧之间使用LK进行特征点的跟踪。然后进行下一次循环。

```C++
if ( color.data==nullptr || depth.data==nullptr )
	continue;
```

> 判断color与depth两个Mat类变量中数据存储区data是否为空指针,来判断是否成功找到了本帧所对应的彩色图与深度图。如果有一项为空，则continue进行下一次循环。

```c++
vector<cv::Point2f> next_keypoints; 
vector<cv::Point2f> prev_keypoints;
for ( auto kp:keypoints )
    prev_keypoints.push_back(kp);
```

> 定义两个存储Point2f类的容器next_keypoints与prev_keypoints，分别用来存储当前帧（下一帧）通过光流跟踪得到的特征点的像素坐标，与前一帧特征点的像素坐标。其中，前一帧的特征点需要将存储特征点的list进行遍历（每次光流跟踪后会有坏点剔除），分别存入prev_keypoints。

```c++
chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
...
chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
```

> chrono是c++11中新添加的时间库。
>
> chrono::time_point 表示一个具体时间。
>
> chrono中steady_clock是当前系统实现的一个维定时钟，该时钟的每个时间嘀嗒单位是均匀的(即长度相等)。
>
> chrono::steady_clock::now()是获取当前时钟。
>
> 参考链接：
>
> https://blog.csdn.net/u013390476/article/details/50209603
>
> https://www.cnblogs.com/jwk000/p/3560086.html

```c++
chrono::duration<double> time_used = 
    					chrono::duration_cast<chrono::duration<double>>( t2-t1 );
```

> std::chrono::duration 表示一段时间，比如两个小时，12.88秒，半个时辰，一炷香的时间等等，只要能换算成秒即可。也就是计算t1到t2这段时间。
>
> ```c++
> template <class Rep, class Period = ratio<1> > class duration;
> ```

```c++
cv::calcOpticalFlowPyrLK( last_color, color, prev_keypoints, next_keypoints, status, error );
```

> 调用OpenCV提供的光流法计算函数calcOpticalFlowPyrLK，通过金字塔LK光流法计算当前帧跟踪得到的特征点的像素坐标并存入next_keypoints，同时会将每一个特征点的跟踪情况存入同维度的容器status与error，用来判断该特征点是否被成功跟踪。

```c++
int i=0; 
for ( auto iter=keypoints.begin(); iter!=keypoints.end(); i++)
{
    if ( status[i] == 0 )
    {
        iter = keypoints.erase(iter);
        continue;
    }
    *iter = next_keypoints[i];
    iter++;
}
```

> 这段代码主要是用来删除跟丢的点。
>
> 创建一个链表keypoints的迭代器iter，依次访问内部元素，通过判断status容器内同位置的标志量是否为0，来选择是否在链表内部删除该特征点。若如果没有跟丢，则使用当前帧该特征点运动到的像素位置替换keypoints中该特征点存储的像素位置（即在前一帧的位置）。



## 二.代码框架总结

### <1> 双目图像特征匹配

- 1.通过opencv读取两张图片，这两张图片是通过轻微移动而成的。

- 2.初始化存储特征点数据的变量

- 3.提取每张图像的FAST角点

- 4.计算每张图像每个FAST角点的BRIEF描述子

- 5.根据刚刚计算好的BRIEF描述子，对两张图的角点进行匹配

- 6.筛去匹配度较差的角点配对

- 7.绘制匹配结果

### <2> ICP 法相机姿态估计

- 1.读取图像
- 2.特征点匹配，存储特征点的坐标用以进行后续的位姿求解
- 3.联合深度信息来构造特征点在当前相机坐标系下的3d坐标
- 4.调用子函数pose_estimation_3d3d进行R、t的求取，其中用到ICP方法。
- 5.最后调用BA进行最小二乘优化

### <3> 光流特征跟踪

- 1.读取图像并提取图像第一帧的特征点。

- 2.对其他帧用金字塔LK光流法计算当前帧跟踪得到的特征点的像素坐标。

- 3.删去跟丢的点。

- 4.画出 keypoints。



## 三.知识点总结

### <1> 典型特征点概念

特征点主要是图像中一些特别的地方，比如角点、边缘和区块。

特征点的性质有四：

- **可重复性**：相同的区域可以在两张图片中都找到。
- **可区别性**：不同的区域有不同的表达。
- **高效率**：同一张图中，特征点的数量远小于像素的数量。
- **本地性**：特征仅与一小片图像区域有关。

### <2> 特征点的实现方法

#### 1.关键点的提取。

> 以ORB为例子，则提取的关键点为FAST角点，其检测过程如下：
>
> 1.选取图像中的某个像素p，假设亮度为Ip。
>
> 2.设置阈值T。
>
> 3.以p为中心选区半径为3的圆上的16个像素点。
>
> 4.如果以p为圆心选取的圆上有连续N个点的亮度大于Ip+T或小于Ip-T ，则认为该点p为关键点。
>
> 5.对所有的像素点执行上面的4步操作。

#### 2.描述子的计算。

> 对提取的关键点的周围信息进行描述。

### <3> ICP算法实现

ICP算是迭代最近点的算法，主要的思想是通过两张稍微平移而得的图像中的某一点的误差最小来估计位姿。ICP算法适用于两张图片都是3D的形式，即3D-3D位姿估计问题。显然，这里面不涉及相机模型，因此在激光SLAM中会涉及到，而在视觉中我们可以通过图像得到很好的匹配关系，通过ICP可以很好估计相机的位姿。

ICP通过最小二乘法来表示两点的误差，把与R,t无关的项丢弃，只保留与R,t相关的项，将问题一步步拆解成简单的形式，最终可以通过SVD的方法或者非线性优化的方法来求解R,t。

> **范数的理解：**
>
> 参考链接：https://blog.csdn.net/SusanZhang1231/article/details/52127011

### <4> 光流法实现

#### 1.基本概念

不通过特征计算视觉里程计：通过其他方式寻找匹配点或无匹配点，分别对应**光流法**或**直接法**。

光流是一种描述像素随着时间，在图像之间运动的方法。随着时间的经过，同一个像素会在图像中运动，而我们希望追踪它的运动过程。计算部分像素运动的称为**稀疏光流**，计算所有像素的称为**稠密光流**。

光流是空间运动物体在观测成像平面上的像素运动的“瞬时速度”。光流的研究是利用图像序列中的像素强度数据的时域变化和相关性来确定各自像素位置的“运动”。研究光流场的目的就是为了从图片序列中近似得到不能直接得到的运动场。

#### 2.光流法的前提假设

（1）相邻帧之间的亮度恒定；

（2）相邻视频帧的取帧时间连续，或者，相邻帧之间物体的运动比较“微小”；

（3）保持空间一致性；即，同一子图像的像素点具有相同的运动

#### 3.光流法用于目标检测的原理

给图像中的每个像素点赋予一个速度矢量，这样就形成了一个运动矢量场。在某一特定时刻，图像上的点与三维物体上的点一一对应，这种对应关系可以通过投影来计算得到。根据各个像素点的速度矢量特征，可以对图像进行动态分析。如果图像中没有运动目标，则光流矢量在整个图像区域是连续变化的。当图像中有运动物体时，目标和背景存在着相对运动。运动物体所形成的速度矢量必然和背景的速度矢量有所不同，如此便可以计算出运动物体的位置。需要提醒的是，利用光流法进行运动物体检测时，计算量较大，无法保证实时性和实用性。

#### 4.光流法用于目标跟踪的原理

（1）对一个连续的视频帧序列进行处理；

（2）针对每一个视频序列，利用一定的目标检测方法，检测可能出现的前景目标；

（3）如果某一帧出现了前景目标，找到其具有代表性的关键特征点（可随机产生，也可利用角点来做特征点）；

（4）对之后的任意两个相邻视频帧而言，寻找上一帧中出现的关键特征点在当前帧中的最佳位置，从而得到前景目标在当前帧中的位置坐标；

（5）如此迭代进行，便可实现目标的跟踪；


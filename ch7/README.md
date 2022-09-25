## 公式错误P183
原式(7.34)：
$$\ (1-u)y^2 - ux^2 - cos<b, c>y + 2uxycos<a,b> + 1=0 $$

$ (1-w)x^2 - wy^2 - \cos<a, c>x + 2wxy\cos<a,b> + 1=0 $
或许该改为：
$ (1-u)y^2 - ux^2 - 2\cos<b, c>y + 2uxy\cos<a,b> + 1=0 $

$ (1-w)x^2 - wy^2 - 2\cos<a, c>x + 2wxy\cos<a,b> + 1=0 $
## g2o优化chi2过高的问题
在ch7视觉里程计1的学习中，基于书上的3D-2D:PnP的BA优化相机位姿方法(g2o)，我尝试了用g2o同时优化相机位姿和特征空间点位置，我参照ch9的CMakeList.txt对编译内容做了修改，然后在pose_estimation_3d2d_re.cpp中include以下.h文件:
```python
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/robust_kernel_impl.h>
```

参考pose_estimation_3d2d.cpp，在pose_estimation_3d2d_re.cpp中首先定义了另外一个节点类VertexPoint(特征点空间位置)(与ch9相同):
```python
class VertexPoint : public g2o::BaseVertex<3, Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoint() {}

    virtual void setToOriginImpl() override {
        _estimate = Vector3d(0, 0, 0);
    }

    virtual void oplusImpl(const double *update) override {
        _estimate += Vector3d(update[0], update[1], update[2]);
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};
```

随后，我重写了g2o的边为EdgeProjectionBoth类(参考建议后先除深度再乘内参矩阵):
```python
class EdgeProjectionBoth :
        public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPose, VertexPoint> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectionBoth(const Eigen::Matrix3d &K) : _K(K) {}

    virtual void computeError() override {
//        const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
        auto v0 = (VertexPose *) _vertices[0];
        auto v1 = (VertexPoint *) _vertices[1];
        Sophus::SE3d T = v0->estimate();
        Eigen::Vector3d pos = T * v1->estimate();
        pos /= pos[2];
        Eigen::Vector3d pos_pixel = _K * pos;
//        cout << "pos_pixel is:\n" << pos_pixel << endl;
//        Eigen::Vector3d pos_pixel = _K * (T * v1->estimate());
//        pos_pixel /= pos_pixel[2];
//        cout << "pos_pixel is:\n" << pos_pixel << endl;
        _error = _measurement - pos_pixel.head<2>();
    }

    // use numeric derivatives
    virtual bool read(istream &in) override {}

    virtual bool write(ostream &out) const override {}

private:
    Eigen::Matrix3d _K;
};
```

最后，修改了g2o的BA优化为函数bundleAdjustmentG2O_re(),修改部分如下:
```python
// vertexpose
VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
vertex_pose->setId(0);
vertex_pose->setEstimate(Sophus::SE3d());
optimizer.addVertex(vertex_pose);
// K
Eigen::Matrix3d K_eigen;
K_eigen <<
        K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
        K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
        K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);

// edges and vertexpoint
int index = 1;
for (size_t i = 0; i < points_2d.size(); ++i) {
    auto p2d = points_2d[i];
    auto p3d = points_3d[i];
    // set vertexpoint
    VertexPoint *vertex_point = new VertexPoint(); // landmark point
    vertex_point->setId(index);
    vertex_point->setEstimate(Eigen::Vector3d(p3d[0], p3d[1], p3d[2]));
    optimizer.addVertex(vertex_point);
    // set edges
    EdgeProjectionBoth *edge = new EdgeProjectionBoth(K_eigen);
    edge->setVertex(0, vertex_pose);
    edge->setVertex(1, vertex_point);
    edge->setMeasurement(p2d);
    edge->setInformation(Eigen::Matrix2d::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber());
    optimizer.addEdge(edge);
    index++;
}
```

修改完了，在编译运行后的结果中，重写的g20优化在iteration=0的时候便停止了，在加了Huber的情况下chi2的值还是非常大，输出情况为:
```
-- Max dist : 95.000000 
-- Min dist : 7.000000 
一共找到了81组匹配点
3d-2d pairs: 77
solve pnp in opencv cost time: 0.000574265 seconds.
R=
[0.9979193252225089, -0.05138618904650331, 0.03894200717386666;
 0.05033852907733834, 0.9983556574295412, 0.02742286944793203;
 -0.04028712992734059, -0.02540552801469367, 0.9988651091656532]
t=
[-0.1255867099750398;
 -0.007363525258777434;
 0.06099926588678889]
calling bundle adjustment by gauss newton
iteration 0 cost=44765.3537799
iteration 1 cost=431.695366816
iteration 2 cost=319.560037493
iteration 3 cost=319.55886789
pose by g-n: 
   0.997919325221  -0.0513861890122   0.0389420072614   -0.125586710123
  0.0503385290405    0.998355657431   0.0274228694545 -0.00736352527141
 -0.0402871300142  -0.0254055280183    0.998865109162   0.0609992659219
                0                 0                 0                 1
solve pnp by gauss newton cost time: 0.000434819 seconds.
calling bundle adjustment by g2o
optimization costs time: 0.001680506 seconds.
pose estimated by g2o_re =
                  1                   0                   0   6.9524532287e-310
                  0                   1 -4.64766124177e-310   6.9524532287e-310
                  0  4.64766124177e-310                   1  4.64766124177e-310
                  0                   0                   0                   1
solve pnp by g2o cost time: 0.0021648 seconds.
iteration= 0	 chi2= 3367.759741	 time= 0.000997457	 cumTime= 0.000997457	 edges= 77	 schur= 0
```
八成是chi2过大导致优化提前终止，请问是我前面g20图的节点、边的构造出了问题吗？有没有大佬看到了能解答一下。

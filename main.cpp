#include <msgpack.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <msgpack_extension/pcl.hpp>

class myclass {
public:
    std::string m_str;
    std::vector<int> m_vec;
    Eigen::Vector3f m_eigenvec;
    pcl::PointCloud<pcl::PointXYZ> m_cloud;
    MSGPACK_DEFINE(m_str, m_vec, m_eigenvec, m_cloud);
};

int main(void) {
    std::vector<myclass> vec;
    // add some elements into vec...
    myclass h;
    h.m_str = "hogehoge";
    h.m_vec.push_back(1);
    h.m_vec.push_back(2);
    h.m_vec.push_back(3);
    h.m_eigenvec(0) = 1;
    h.m_eigenvec(1) = 2;
    h.m_eigenvec(2) = 3;
    h.m_cloud.resize(10);
    h.m_cloud.width = 10;
    h.m_cloud.height = 1;
    for (size_t i = 0; i < 10; i++) {
        h.m_cloud.points[i].x = i*0.1;
        h.m_cloud.points[i].y = i*0.2;
        h.m_cloud.points[i].z = i*0.3;
    }
    vec.push_back(h);

    // you can serialize myclass directly
    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, vec);

    msgpack::unpacked msg;
    msgpack::unpack(&msg, sbuf.data(), sbuf.size());

    msgpack::object obj = msg.get();

    // you can convert object to myclass directly
    std::vector<myclass> rvec;
    obj.convert(&rvec);
    for (auto& var : rvec) {
        std::cout << var.m_str << std::endl;
        for (auto& var2 : var.m_vec) {
            std::cout << var2 << std::endl;
        }
        for (size_t i = 0; i < 3; i++) {
            std::cout << var.m_eigenvec(i) << std::endl;
        }
        std::cout << "points size: " << var.m_cloud.points.size() << std::endl;
        std::cout << "width: " << var.m_cloud.width << std::endl;
        std::cout << "height: " << var.m_cloud.height << std::endl;
        for (size_t i = 0; i < var.m_cloud.points.size(); i++) {
            std::cout << var.m_cloud.points[i].x << " " << var.m_cloud.points[i].y << " " << var.m_cloud.points[i].z <<std::endl;
        }
    }
}


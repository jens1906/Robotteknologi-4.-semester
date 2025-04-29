#include <QApplication>
#include <QVBoxLayout>
#include <QLabel>
#include <QWidget>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DRender/QCamera>  // Include the full definition of QCamera
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <thread>
#include <QQuaternion>

class Drone3DView : public QWidget {
    Q_OBJECT

public:
    Drone3DView() {
        setWindowTitle("Drone 3D View");
        resize(800, 600);

        auto layout = new QVBoxLayout(this);

        // Create a Qt3D window
        view_ = new Qt3DExtras::Qt3DWindow();
        container_ = QWidget::createWindowContainer(view_);
        layout->addWidget(container_);

        // Root entity
        root_entity_ = new Qt3DCore::QEntity();

        // Create a simple 3D model (e.g., a cone to represent the drone)
        auto drone_entity = new Qt3DCore::QEntity(root_entity_);
        auto cone_mesh = new Qt3DExtras::QConeMesh();
        cone_mesh->setTopRadius(0.0f);
        cone_mesh->setBottomRadius(0.5f);
        cone_mesh->setLength(1.0f);
        drone_entity->addComponent(cone_mesh);

        // Add material to the drone
        auto material = new Qt3DExtras::QPhongMaterial();
        material->setDiffuse(QColor(0, 255, 0));
        drone_entity->addComponent(material);

        // Add transform to the drone
        drone_transform_ = new Qt3DCore::QTransform();
        drone_entity->addComponent(drone_transform_);

        // Set up the camera
        auto camera = view_->camera();
        camera->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 0.1f, 100.0f);
        camera->setPosition(QVector3D(0, 0, 5));
        camera->setViewCenter(QVector3D(0, 0, 0));

        // Add camera controller
        auto cam_controller = new Qt3DExtras::QOrbitCameraController(root_entity_);
        cam_controller->setCamera(camera);

        // Set the root entity
        view_->setRootEntity(root_entity_);
    }

    void updateDroneOrientation(const QQuaternion &quaternion) {
        drone_transform_->setRotation(quaternion);
    }

private:
    Qt3DExtras::Qt3DWindow *view_;
    QWidget *container_;
    Qt3DCore::QEntity *root_entity_;
    Qt3DCore::QTransform *drone_transform_;
};

class BatteryStatusGUI : public QWidget {
    Q_OBJECT

public:
    BatteryStatusGUI() {
        setWindowTitle("Battery Status");
        resize(300, 100);

        auto layout = new QVBoxLayout(this);
        voltage_label_ = new QLabel("Battery Voltage: -- V", this);
        layout->addWidget(voltage_label_);
    }

    void updateVoltage(float voltage) {
        voltage_label_->setText(QString("Battery Voltage: %1 V").arg(voltage, 0, 'f', 2));
    }

private:
    QLabel *voltage_label_;
};

void rosSpinThread(rclcpp::Node::SharedPtr node, BatteryStatusGUI *battery_gui, Drone3DView *drone_view) {
    // Subscribe to battery status
    auto battery_subscription = node->create_subscription<px4_msgs::msg::BatteryStatus>(
        "/fmu/out/battery_status", rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
        [battery_gui](const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
            QMetaObject::invokeMethod(battery_gui, [battery_gui, msg]() {
                battery_gui->updateVoltage(msg->voltage_v);
            });
        });

    // Subscribe to vehicle attitude
    auto attitude_subscription = node->create_subscription<px4_msgs::msg::VehicleAttitude>(
        "/fmu/out/vehicle_attitude", rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT),
        [drone_view](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
            QMetaObject::invokeMethod(drone_view, [drone_view, msg]() {
                // Convert PX4 quaternion to Qt quaternion
                QQuaternion quaternion(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
                drone_view->updateDroneOrientation(quaternion);
            });
        });

    rclcpp::spin(node);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    // Create the battery status GUI
    BatteryStatusGUI battery_gui;
    battery_gui.show();

    // Create the 3D drone view
    Drone3DView drone_view;
    drone_view.show();

    // Create the ROS 2 node
    auto node = rclcpp::Node::make_shared("battery_status_and_drone_view_gui");

    // Start the ROS 2 spin thread
    std::thread ros_thread(rosSpinThread, node, &battery_gui, &drone_view);

    // Run the Qt application
    int ret = app.exec();

    // Shutdown ROS 2 and join the thread
    rclcpp::shutdown();
    ros_thread.join();

    return ret;
}

#include "gui.moc"

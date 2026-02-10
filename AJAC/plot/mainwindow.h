//
// Created by lingwei on 10/30/24.
//

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>

#include "qcustomplot.h"
#include "usb_interface.h"
#include "motor_keeper.h"

QT_BEGIN_NAMESPACE

namespace Ui {
    class MainWindow;
}

enum motor_state {
    MOTOR_DISABLE = 0,
    MOTOR_ENABLE = 1,
    MOTOR_CALI = 2,
};

enum usb_state {
    usb_disconnect = 0,
    usb_connect = 1,
    usb_run = 2,
    usb_collect = 3,
};

QT_END_NAMESPACE

class MainWindow : public QWidget {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow() override;

    void SetupFigures();

    static void setupRealtimeData(QCustomPlot *customPlot);

private slots:
    // slots
    void realtimeDataSlot();

    void Update_Cmd_Slot();

    // buttons
    void IdListIndex(int index);

    void EnableButton_clicked();

    void DisableButton_clicked();

    void CaliButton_clicked();

    void ZeroButton_clicked();

    void ZeroAllButton_clicked();

    void CaliDoneButton_clicked();

    void ConnectButton_clicked();

    void ControlALGButton_clicked();

    void SampleDataButton_clicked();

    void StopSampleButton_clicked();

    void Control_DataButton_clicked();

    void StopControlButton_clicked();

    // text update
    void Product_id_textChanged();

    void Vendor_id_textChanged();

    void Pos_textChanged();
    void Vel_textChanged();
    void Torque_textChanged();
    void Kp_textChanged();
    void Kd_textChanged();

private:
    Ui::MainWindow *ui;
    QTimer dataTimer;
    // ************************************* motor parameters *************************************
    int select_motor_id_ = 0;
    uint16_t usb_product_id_ = 0x2222;
    uint16_t usb_vendor_id_ = 0x1111;
    motor_state motor_state_ = MOTOR_DISABLE;

    // ************************************* usb parameters *************************************
    USB2CAN_Board *spUSB2CAN = nullptr;
    spi_command_t *usb_cmd_from_controller_ = nullptr;
    spi_data_t *usb_data_to_controller_ = nullptr;
    const uint8_t motors_ep_in_ = 0x81; // only this value
    const uint8_t motors_ep_out_ = 0x01; // only this value
    usb_state usb_state_ = usb_disconnect;
    float q, dq, tau, qr, dqr, taur, taur_last; // selected motor datas
    float uq_r, ud_r, uq_esti; // selected motor datas
    float ctrl_kp_ = 0, ctrl_kd_ = 0, ctrl_q_des_ = 0, ctrl_qd_des_ = 0, ctrl_tau_ff_ = 0;
    float fake_torque = 0;

    void USB2CAN_SetCmd();

    long iter = 0;
    int control_button_clicked = 0;
    bool control_alg_cond_ = false;
    std::chrono::steady_clock::time_point time_last_in;
    std::chrono::steady_clock::time_point time_now_in;
    // *********************************** Motor Keeper ******************************************
    MotorKeeper *spMotorKeeper = nullptr;
    std::thread cmd_thread_;
};


#endif //MAINWINDOW_H

//
// Created by lingwei on 10/30/24.
//

// You may need to build the project (run Qt uic code generator) to get "ui_MainWindow.h" resolved

#include "../inc/mainwindow.h"
#include "../inc/qcustomplot.h"
#include "../form/ui_mainwindow.h"
#include <iostream>
#include <unistd.h>
#include <sys/timerfd.h>

MainWindow::MainWindow(QWidget *parent) : QWidget(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    QStringList list = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11"};
    spMotorKeeper = new MotorKeeper();

    ui->idlist->addItems(list);
    connect(ui->idlist, SIGNAL(currentIndexChanged(int)), this, SLOT(IdListIndex(int)));
    // ui->pushButton
    // button connect
    connect(ui->Enable, SIGNAL(clicked()), this, SLOT(EnableButton_clicked()));
    connect(ui->Calibration, SIGNAL(clicked()), this, SLOT(CaliButton_clicked()));
    connect(ui->Disable, SIGNAL(clicked()), this, SLOT(DisableButton_clicked()));
    connect(ui->Zero, SIGNAL(clicked()), this, SLOT(ZeroButton_clicked()));
    connect(ui->Zero_All, SIGNAL(clicked()), this, SLOT(ZeroAllButton_clicked()));
    connect(ui->Cali_Done, SIGNAL(clicked()), this, SLOT(CaliDoneButton_clicked()));
    connect(ui->Con_USB, SIGNAL(clicked()), this, SLOT(ConnectButton_clicked()));
    connect(ui->Control_Alg, SIGNAL(clicked()), this, SLOT(ControlALGButton_clicked()));
    connect(ui->Sample_Data, SIGNAL(clicked()), this, SLOT(SampleDataButton_clicked()));
    connect(ui->Stop_Sample, SIGNAL(clicked()), this, SLOT(StopSampleButton_clicked()));
    connect(ui->Control, SIGNAL(clicked()), this, SLOT(Control_DataButton_clicked()));
    connect(ui->Stop_Control, SIGNAL(clicked()), this, SLOT(StopControlButton_clicked()));

    // editText
    connect(ui->Pro_ID, SIGNAL(returnPressed()), this, SLOT(Product_id_textChanged()));
    connect(ui->Ven_ID, SIGNAL(returnPressed()), this, SLOT(Vendor_id_textChanged()));
    connect(ui->Pos_input,SIGNAL(returnPressed()), this, SLOT(Pos_textChanged()));
    connect(ui->Vel_Input, SIGNAL(returnPressed()), this, SLOT(Vel_textChanged()));
    connect(ui->Torque_input, SIGNAL(returnPressed()), this, SLOT(Torque_textChanged()));
    connect(ui->kp_input, SIGNAL(returnPressed()), this, SLOT(Kp_textChanged()));
    connect(ui->kd_input, SIGNAL(returnPressed()), this, SLOT(Kd_textChanged()));

    QString default_pro_id = "Default: 0x2222";
    QString default_ven_id = "Default: 0x1111";
    ui->Pro_ID->setText(default_pro_id);
    ui->Ven_ID->setText(default_ven_id);

    SetupFigures();
}

MainWindow::~MainWindow() {
    delete ui;
    delete spMotorKeeper;
}

void MainWindow::SetupFigures() {
    // generate some data:
    setupRealtimeData(ui->Pos);
    setupRealtimeData(ui->Velocity);
    setupRealtimeData(ui->Torque);
    setupRealtimeData(ui->Uq_plot);
    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(&dataTimer, SIGNAL(timeout()), this, SLOT(realtimeDataSlot()));
    dataTimer.start(4); // Interval 0 means to refresh as fast as possible
    ui->Pos->graph(0)->setName("Ud");
    ui->Pos->graph(1)->setName("Ctrl_tau");
    ui->Pos->plotLayout()->addElement(
        0, 0, new QCPTextElement(ui->Pos, "Torque_Left", QFont("sans", 12, QFont::Bold)));
    ui->Velocity->graph(0)->setName("qd_ref");
    ui->Velocity->graph(1)->setName("qd");
    ui->Velocity->plotLayout()->addElement(
        0, 0, new QCPTextElement(ui->Velocity, "Motor Velocity", QFont("sans", 12, QFont::Bold)));
    ui->Torque->graph(0)->setName("tau_des");
    ui->Torque->graph(1)->setName("tau");
    ui->Torque->plotLayout()->addElement(
        0, 0, new QCPTextElement(ui->Torque, "Motor Torque", QFont("sans", 12, QFont::Bold)));

    ui->Uq_plot->graph(0)->setName("Uq");
    ui->Uq_plot->graph(1)->setName("Uq_esti");
    ui->Uq_plot->plotLayout()->addElement(
        0, 0, new QCPTextElement(ui->Uq_plot, "D-Q Voltage", QFont("sans", 12, QFont::Bold)));

    QStatusBar().clearMessage();
    ui->Pos->replot();
}

void MainWindow::setupRealtimeData(QCustomPlot *customPlot) {
    // include this section to fully disable antialiasing for higher performance:
    /*
    customPlot->setNotAntialiasedElements(QCP::aeAll);
    QFont font;
    font.setStyleStrategy(QFont::NoAntialias);
    customPlot->xAxis->setTickLabelFont(font);
    customPlot->yAxis->setTickLabelFont(font);
    customPlot->legend->setFont(font);
    */
    customPlot->legend->setVisible(true);
    customPlot->plotLayout()->insertRow(0);
    customPlot->addGraph(); // blue line
    QPen reference_pen;
    reference_pen.setColor(QColor(40, 110, 255));
    reference_pen.setWidth(4);
    customPlot->graph(0)->setPen(reference_pen);

    QPen real_penl;
    real_penl.setColor(QColor(255, 110, 40));
    real_penl.setWidth(4);
    customPlot->addGraph(); // red line
    customPlot->graph(1)->setPen(real_penl);

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");
    customPlot->xAxis->setTicker(timeTicker);
    customPlot->axisRect()->setupFullAxesBox();
    customPlot->yAxis->setRange(-1.2, 1.2);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));
}

void MainWindow::realtimeDataSlot() {
    if (usb_state_ == usb_collect) {
        static QTime timeStart = QTime::currentTime();
        // calculate two new data points:
        double key = timeStart.msecsTo(QTime::currentTime()) / 1000.0; // time elapsed since start of demo, in seconds
        static double lastPointKey = 0;

        // ensure the data;
        int leg_index = select_motor_id_ / 3;
        int id_index = select_motor_id_ % 3;
        // spUSB2CAN->lock_in_mutex();
        // spUSB2CAN->lock_out_mutex();
        switch (id_index) {
            case 0:
                q = usb_data_to_controller_->q_abad[leg_index];
                dq = usb_data_to_controller_->qd_abad[leg_index];
                tau = usb_data_to_controller_->tau_abad[leg_index];
                qr = usb_cmd_from_controller_->q_des_abad[leg_index];
                dqr = usb_cmd_from_controller_->qd_des_abad[leg_index];
                taur = usb_cmd_from_controller_->tau_abad_ff[leg_index];
                uq_r = usb_data_to_controller_->uq_abad[leg_index];
                ud_r = usb_data_to_controller_->ud_abad[leg_index];
                break;
            case 1:
                q = usb_data_to_controller_->q_hip[leg_index];
                dq = usb_data_to_controller_->qd_hip[leg_index];
                tau = usb_data_to_controller_->tau_hip[leg_index];
                qr = usb_cmd_from_controller_->q_des_hip[leg_index];
                dqr = usb_cmd_from_controller_->qd_des_hip[leg_index];
                taur = usb_cmd_from_controller_->tau_hip_ff[leg_index];
                uq_r = usb_data_to_controller_->uq_hip[leg_index];
                ud_r = usb_data_to_controller_->ud_hip[leg_index];
                break;
            case 2:
                q = usb_data_to_controller_->q_knee[leg_index];
                dq = usb_data_to_controller_->qd_knee[leg_index];
                tau = usb_data_to_controller_->tau_knee[leg_index];
                qr = usb_cmd_from_controller_->q_des_knee[leg_index];
                dqr = usb_cmd_from_controller_->qd_des_knee[leg_index];
                taur = usb_cmd_from_controller_->tau_knee_ff[leg_index];
                uq_r = usb_data_to_controller_->uq_knee[leg_index];
                ud_r = usb_data_to_controller_->ud_knee[leg_index];
                break;
            default:
                break;
        }

        // spUSB2CAN->unlock_in_mutex();
        // spUSB2CAN->unlock_out_mutex();
        uq_esti = spMotorKeeper->get_uq_esti(select_motor_id_);
        // std::cout << "Pos data: ID " << select_motor_id_ << " " << q << " " << qr << "\n";

        if (key - lastPointKey > 0.002) // at most add point every 2 ms
        {
            // add data to lines:
            ui->Pos->graph(0)->addData(key, ud_r);
            ui->Pos->graph(1)->addData(key, fake_torque);
            // rescale value (vertical) axis to fit the current data:
            ui->Pos->graph(0)->rescaleValueAxis();
            ui->Pos->graph(1)->rescaleValueAxis(true);

            ui->Velocity->graph(0)->addData(key, dqr);
            ui->Velocity->graph(1)->addData(key, dq);
            // rescale value (vertical) axis to fit the current data:
            ui->Velocity->graph(0)->rescaleValueAxis();
            ui->Velocity->graph(1)->rescaleValueAxis(true);

            ui->Torque->graph(0)->addData(key, taur_last);
            ui->Torque->graph(1)->addData(key, tau);
            // rescale value (vertical) axis to fit the current data:
            ui->Torque->graph(0)->rescaleValueAxis();
            ui->Torque->graph(1)->rescaleValueAxis(true);

            ui->Uq_plot->graph(0)->addData(key, uq_r);
            ui->Uq_plot->graph(1)->addData(key, uq_esti);
            // rescale value (vertical) axis to fit the current data:
            ui->Uq_plot->graph(0)->rescaleValueAxis();
            ui->Uq_plot->graph(1)->rescaleValueAxis(true);
            lastPointKey = key;
        }
        // make key axis range scroll with the data (at a constant range size of 8):
        ui->Pos->xAxis->setRange(key, 8, Qt::AlignRight);
        ui->Pos->replot();
        ui->Velocity->xAxis->setRange(key, 8, Qt::AlignRight);
        ui->Velocity->replot();
        ui->Torque->xAxis->setRange(key, 8, Qt::AlignRight);
        ui->Torque->replot();
        ui->Uq_plot->xAxis->setRange(key, 8, Qt::AlignRight);
        ui->Uq_plot->replot();
        taur_last = taur; // update torque_cmd, due to the lack of collecting data.
    }
}

void MainWindow::Update_Cmd_Slot() {
    // ctrl_kp_ = 0;
    // ctrl_kd_ = 0;
    // ctrl_q_des_ = 0;
    // ctrl_qd_des_ = 0;
    // ctrl_tau_ff_ = -1.f;

    int timerfd = timerfd_create(CLOCK_MONOTONIC, 0);;
    itimerspec timeSpec{};
    timeSpec.it_interval.tv_sec = 0;
    timeSpec.it_value.tv_sec = 0;
    timeSpec.it_value.tv_nsec = 1000000; //1ms
    timeSpec.it_interval.tv_nsec = 1000000; // 1ms
    timerfd_settime(timerfd, 0, &timeSpec, nullptr);

    int leg_index = select_motor_id_ / 3;
    int id_index = select_motor_id_ % 3;

    while (true) {

        if(control_alg_cond_) {
            // update reference by code
            ctrl_tau_ff_ = 2*sinf(iter * 0.01);
        }

        float ud_in, qd_in, uq_in;
        switch (id_index) {
            case 0:
                ud_in = usb_data_to_controller_->ud_abad[leg_index];
                qd_in = usb_data_to_controller_->qd_abad[leg_index];
                uq_in = usb_data_to_controller_->uq_abad[leg_index];
                break;
            case 1:
                ud_in = usb_data_to_controller_->ud_hip[leg_index];
                qd_in = usb_data_to_controller_->qd_hip[leg_index];
                uq_in = usb_data_to_controller_->uq_hip[leg_index];
                break;
            case 2:
                ud_in = usb_data_to_controller_->ud_knee[leg_index];
                qd_in = usb_data_to_controller_->qd_knee[leg_index];
                uq_in = usb_data_to_controller_->uq_knee[leg_index];
                break;
            default:
                break;
        }

        spUSB2CAN->lock_in_mutex();
        fake_torque = spMotorKeeper->Update(select_motor_id_, ud_in, uq_in, qd_in, ctrl_tau_ff_);
        spUSB2CAN->unlock_in_mutex();

        spUSB2CAN->lock_out_mutex();
        switch (id_index) {
            case 0:
                usb_cmd_from_controller_->kd_abad[leg_index] = ctrl_kd_;
                usb_cmd_from_controller_->kp_abad[leg_index] = ctrl_kp_;
                usb_cmd_from_controller_->tau_abad_ff[leg_index] = fake_torque;
                usb_cmd_from_controller_->q_des_abad[leg_index] = ctrl_q_des_;
                usb_cmd_from_controller_->qd_des_abad[leg_index] = ctrl_qd_des_;
                break;
            case 1:
                usb_cmd_from_controller_->kd_hip[leg_index] = ctrl_kd_;
                usb_cmd_from_controller_->kp_hip[leg_index] = ctrl_kp_;
                usb_cmd_from_controller_->tau_hip_ff[leg_index] = fake_torque;
                usb_cmd_from_controller_->q_des_hip[leg_index] = ctrl_q_des_;
                usb_cmd_from_controller_->qd_des_hip[leg_index] = ctrl_qd_des_;
                break;
            case 2:
                usb_cmd_from_controller_->kd_knee[leg_index] = ctrl_kd_;
                usb_cmd_from_controller_->kp_knee[leg_index] = ctrl_kp_;
                usb_cmd_from_controller_->tau_knee_ff[leg_index] = fake_torque;
                usb_cmd_from_controller_->q_des_knee[leg_index] = ctrl_q_des_;
                usb_cmd_from_controller_->qd_des_knee[leg_index] = ctrl_qd_des_;
                break;
            default:
                break;
        }
        spUSB2CAN->unlock_out_mutex();
        unsigned long long missed = 0;
        int m = read(timerfd, &missed, sizeof(missed));
        (void) m;
        iter++;
        if (usb_state_ == usb_connect)
            break;
        // time_now_in = std::chrono::steady_clock::now();
        // std::chrono::duration<double, std::micro> time_used = std::chrono::duration_cast<std::chrono::duration<
        //     double,
        //     std::micro> >(time_now_in - time_last_in);
        // std::cout << "[Time Interval Out]: " << time_used.count() << " us\n";
        // time_last_in = time_now_in;
    }
}

void MainWindow::IdListIndex(int index) {
    select_motor_id_ = index;
    QString info = "Current motor id: " + QString::number(index);
    ui->textBrowser->append(info);
}

void MainWindow::EnableButton_clicked() {
    if (motor_state_ == MOTOR_DISABLE) {
        if (select_motor_id_ <= 11 && select_motor_id_ >= 0) {
            memset(usb_cmd_from_controller_, 0, sizeof(spi_command_t));
            int chip_id = select_motor_id_ / 6;
            usb_cmd_from_controller_->flags[chip_id] = (0x01 << 2) | 0x01;
            spUSB2CAN->USB2CAN_Start_Transfer_Sync();
            motor_state_ = MOTOR_ENABLE;
            QString info = "Enable motor, ID " + QString::number(select_motor_id_);
            ui->textBrowser->append(info);
        } else {
            QColor color = QColor(255, 0, 0);
            ui->textBrowser->setTextColor(color);
            QString info = "Enable Invalid motor ID";
            ui->textBrowser->append(info);
            color = QColor(0, 0, 0);
            ui->textBrowser->setTextColor(color);
        }
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Motor not disabled, can't Enable";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::DisableButton_clicked() {
    if (motor_state_ != MOTOR_CALI) {
        if (select_motor_id_ <= 11 && select_motor_id_ >= 0) {
            memset(usb_cmd_from_controller_, 0, sizeof(spi_command_t));
            int chip_id = select_motor_id_ / 6;
            usb_cmd_from_controller_->flags[chip_id] = (0x01 << 1);
            spUSB2CAN->USB2CAN_Start_Transfer_Sync();
            motor_state_ = MOTOR_DISABLE;
            usb_state_ = usb_connect;
            QString info = "Disable motor, ID " + QString::number(select_motor_id_);
            ui->textBrowser->append(info);
        } else {
            QColor color = QColor(255, 0, 0);
            ui->textBrowser->setTextColor(color);
            QString info = "Disable Invalid motor ID";
            ui->textBrowser->append(info);
            color = QColor(0, 0, 0);
            ui->textBrowser->setTextColor(color);
        }
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Calirating, can't disable. Please Cali Done first";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::CaliButton_clicked() {
    if (motor_state_ == MOTOR_DISABLE) {
        if (select_motor_id_ <= 11 && select_motor_id_ >= 0) {
            memset(usb_cmd_from_controller_, 0, sizeof(spi_command_t));
            int chip_id = select_motor_id_ / 6;
            int motor_number = select_motor_id_ % 6 + 1;
            usb_cmd_from_controller_->flags[chip_id] = (uint16_t) motor_number << 9;
            spUSB2CAN->USB2CAN_Start_Transfer_Sync();
            motor_state_ = MOTOR_CALI;
            QString info = "Cali motor, ID " + QString::number(select_motor_id_);
            ui->textBrowser->append(info);
        } else {
            QColor color = QColor(255, 0, 0);
            ui->textBrowser->setTextColor(color);
            QString info = "Cali Invalid motor ID";
            ui->textBrowser->append(info);
            color = QColor(0, 0, 0);
            ui->textBrowser->setTextColor(color);
        }
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Motor not disabled, can't Cali";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::ZeroButton_clicked() {
    if (motor_state_ == MOTOR_DISABLE) {
        if (select_motor_id_ <= 11 && select_motor_id_ >= 0) {
            int chip_id = select_motor_id_ / 6;
            int motor_number = select_motor_id_ % 6 + 1;
            memset(usb_cmd_from_controller_, 0, sizeof(spi_command_t));
            usb_cmd_from_controller_->flags[chip_id] = (uint16_t) motor_number << 5;
            spUSB2CAN->USB2CAN_Start_Transfer_Sync();
            QString info = "Zero motor, ID " + QString::number(select_motor_id_);
            ui->textBrowser->append(info);
        } else {
            QColor color = QColor(255, 0, 0);
            ui->textBrowser->setTextColor(color);
            QString info = "Zero Invalid motor ID";
            ui->textBrowser->append(info);
            color = QColor(0, 0, 0);
            ui->textBrowser->setTextColor(color);
        }
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Motor not disabled, can't zero";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::ZeroAllButton_clicked() {
    if (motor_state_ == MOTOR_DISABLE) {
        for (int i = 0; i < 12; i++) {
            int chip_id = i / 6;
            int motor_number = i % 6 + 1;
            usb_cmd_from_controller_->flags[chip_id] = (uint16_t) motor_number << 5;
            spUSB2CAN->USB2CAN_Start_Transfer_Sync();
            std::cout << "Motor Index: " << i << std::endl;
            usleep(100000);
        }
        QString info = "Zero all motors, id range: 0-11";
        ui->textBrowser->append(info);
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Motor not disabled, can't zero";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::CaliDoneButton_clicked() {
    if (motor_state_ == MOTOR_CALI) {
        motor_state_ = MOTOR_DISABLE;
        QString info = "Motor Cali Done";
        ui->textBrowser->append(info);
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Not Cali Yet";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::ConnectButton_clicked() {
    if (usb_product_id_ != 0 && usb_vendor_id_ != 0) {
        if (usb_state_ == usb_disconnect) {
            spUSB2CAN = new USB2CAN_Board(usb_vendor_id_, usb_product_id_, motors_ep_in_, motors_ep_out_);
            usb_cmd_from_controller_ = new spi_command_t();
            usb_data_to_controller_ = new spi_data_t();
            // spUSB2CAN->print_time();
            spUSB2CAN->set_write_variable();
            bool ret = spUSB2CAN->USB2CAN_Connect();
            spUSB2CAN->USB2CAN_SetBuffer(usb_cmd_from_controller_, usb_data_to_controller_);

            if (ret) {
                QColor color = QColor(0, 0, 255);
                ui->textBrowser->setTextColor(color);
                QString info = "USB Connect Success";
                usb_state_ = usb_connect;
                ui->textBrowser->append(info);
                color = QColor(0, 0, 0);
                ui->textBrowser->setTextColor(color);
            } else {
                QColor color = QColor(255, 0, 0);
                ui->textBrowser->setTextColor(color);
                QString info = "USB Connect Failed, Please check USB ID";
                ui->textBrowser->append(info);
                color = QColor(0, 0, 0);
                ui->textBrowser->setTextColor(color);
            }
        } else {
            QString info = "USB Already Connected";
            ui->textBrowser->append(info);
        }
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Invalid USB ID, Input ID first";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::ControlALGButton_clicked() {
    if (usb_state_ == usb_collect) {
        control_button_clicked++;
        if(control_button_clicked % 2 == 0) {
            control_alg_cond_ = false;
            QString info = "Control Algorithm Stop";
            ctrl_kd_ = ctrl_kp_ = ctrl_q_des_ = ctrl_qd_des_ = ctrl_tau_ff_ = 0;
            ui->textBrowser->append(info);
        } else {
            control_alg_cond_ = true;
            QString info = "Control Algorithm Start";
            ctrl_kd_ = ctrl_kp_ = ctrl_q_des_ = ctrl_qd_des_ = ctrl_tau_ff_ = 0;
            ui->textBrowser->append(info);
        }
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Click Control Button First";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::SampleDataButton_clicked() {
    if (usb_state_ == usb_connect) {
        spUSB2CAN->start_usb();
        spUSB2CAN->quit_count = 0;
        memset(usb_cmd_from_controller_, 0, sizeof(spi_command_t));
        usb_cmd_from_controller_->flags[0] = usb_cmd_from_controller_->flags[1] = 0x01 | (0x01 << 2);
        spUSB2CAN->USB2CAN_Start_Transfer_Ans();
        usb_state_ = usb_collect;
        QString info = "Start USB Transfer";
        ui->textBrowser->append(info);
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Connect USB First";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::StopSampleButton_clicked() {
    if (usb_state_ == usb_collect) {
        usb_state_ = usb_connect;
        spUSB2CAN->stop_usb();
        spUSB2CAN->usb_handle_thread.join();
        spUSB2CAN->unlock_in_mutex();
        spUSB2CAN->unlock_out_mutex();
        memset(usb_cmd_from_controller_, 0, sizeof(spi_command_t));
        int chip_id = select_motor_id_ / 6;
        usb_cmd_from_controller_->flags[chip_id] = (0x01 << 1);
        usleep(2000000);
        spUSB2CAN->USB2CAN_Start_Transfer_Sync();
        motor_state_ = MOTOR_DISABLE;

        QString info = "Pause USB Transfer";
        ui->textBrowser->append(info);
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Run USB First";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::Control_DataButton_clicked() {
    if (usb_state_ == usb_connect) {
        spUSB2CAN->start_usb();
        spUSB2CAN->quit_count = 0;
        memset(usb_cmd_from_controller_, 0, sizeof(spi_command_t));
        usb_cmd_from_controller_->flags[0] = usb_cmd_from_controller_->flags[1] = 0x01 | (0x01 << 2);
        spUSB2CAN->USB2CAN_Start_Transfer_Ans();
        // TODO add timer
        cmd_thread_ = std::thread(&MainWindow::Update_Cmd_Slot, this);
        usb_state_ = usb_collect;
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Connect USB First";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::StopControlButton_clicked() {
    if (usb_state_ == usb_collect) {
        usb_state_ = usb_connect;
        spUSB2CAN->stop_usb();
        if (cmd_thread_.joinable())
            cmd_thread_.join();

        spUSB2CAN->usb_handle_thread.join();
        spUSB2CAN->unlock_in_mutex();
        spUSB2CAN->unlock_out_mutex();
        usleep(2000000);
        memset(usb_cmd_from_controller_, 0, sizeof(spi_command_t));
        int chip_id = select_motor_id_ / 6;
        usb_cmd_from_controller_->flags[chip_id] = (0x01 << 1);
        spUSB2CAN->USB2CAN_Start_Transfer_Sync();
        motor_state_ = MOTOR_DISABLE;
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Collect USB First";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::Product_id_textChanged() {
    QColor color = QColor(0, 0, 255);
    ui->textBrowser->setTextColor(color);
    usb_product_id_ = ui->Pro_ID->text().toUShort();
    QString info = "Product ID: " + QString::number(usb_product_id_);
    ui->textBrowser->append(info);
    color = QColor(0, 0, 0);
    ui->textBrowser->setTextColor(color);
}

void MainWindow::Vendor_id_textChanged() {
    QColor color = QColor(0, 0, 255);
    ui->textBrowser->setTextColor(color);
    usb_vendor_id_ = ui->Ven_ID->text().toUShort();
    QString info = "Vendor ID: " + QString::number(usb_vendor_id_);
    ui->textBrowser->append(info);
    color = QColor(0, 0, 0);
    ui->textBrowser->setTextColor(color);
}

void MainWindow::Pos_textChanged() {
    float temp_pos = ui->Pos_input->text().toFloat();
    if (std::abs(temp_pos) < 12.5f) {
        QColor color = QColor(0, 0, 255);
        ui->textBrowser->setTextColor(color);
        ctrl_q_des_ = temp_pos;
        QString info = "Pos: " + QString::number(ctrl_q_des_);
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Invalid Position Input.";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::Vel_textChanged() {
    float temp_vel = ui->Vel_Input->text().toFloat();
    if (std::abs(temp_vel) < 30.f) {
        QColor color = QColor(0, 0, 255);
        ui->textBrowser->setTextColor(color);
        ctrl_qd_des_ = temp_vel;
        QString info = "Vel: " + QString::number(ctrl_qd_des_);
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Invalid Velcity Input.";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::Torque_textChanged() {
    float temp_torque = ui->Torque_input->text().toFloat();
    if (std::abs(temp_torque) < 20.f) {
        QColor color = QColor(0, 0, 255);
        ui->textBrowser->setTextColor(color);
        ctrl_tau_ff_ = temp_torque;
        QString info = "Torque: " + QString::number(ctrl_tau_ff_);
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Invalid Torque Input.";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::Kp_textChanged() {
    float temp_kp = ui->kp_input->text().toFloat();
    if (std::abs(temp_kp) < 500.f) {
        QColor color = QColor(0, 0, 255);
        ui->textBrowser->setTextColor(color);
        ctrl_kp_ = temp_kp;
        QString info = "Kp: " + QString::number(ctrl_kp_);
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Invalid KP Input.";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::Kd_textChanged() {
    float temp_kd = ui->kd_input->text().toFloat();
    if (std::abs(temp_kd) < 5.f) {
        QColor color = QColor(0, 0, 255);
        ui->textBrowser->setTextColor(color);
        ctrl_kd_ = temp_kd;
        QString info = "Kd: " + QString::number(ctrl_kd_);
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    } else {
        QColor color = QColor(255, 0, 0);
        ui->textBrowser->setTextColor(color);
        QString info = "Invalid KD Input.";
        ui->textBrowser->append(info);
        color = QColor(0, 0, 0);
        ui->textBrowser->setTextColor(color);
    }
}

void MainWindow::USB2CAN_SetCmd() {
    spUSB2CAN->lock_out_mutex();
    spUSB2CAN->USB2CAN_Start_Transfer_Sync();
    spUSB2CAN->unlock_out_mutex();
}

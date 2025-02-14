#include <QWidget>
#include <QDialog>
#include <QDialogButtonBox>
#include <QEventLoop>
#include <QGridLayout>
#include <QPushButton>
#include <QSpacerItem>
#include "dialog_factory.h"
#include "gui_color.h"
#include "ui/dialog.h"

using namespace intball;

MainDialog* DialogFactory::operatorInformationDialog = nullptr;
MainDialog* DialogFactory::operatorAlertDialog = nullptr;
MainDialog* DialogFactory::operatorConfirmDialog = nullptr;
CameraMicrophoneSettingsDialog* DialogFactory::cameraMicrophoneSettingsDialog = nullptr;
LedSettingsDialog* DialogFactory::ledSettingsDialog = nullptr;
MainDialog* DialogFactory::tmpDialog = nullptr;


MainDialog& DialogFactory::getMainInformationDialog()
{
    if(operatorInformationDialog == nullptr)
    {
        operatorInformationDialog = new MainDialog();
    }

    return *operatorInformationDialog;
}

MainDialog& DialogFactory::getMainAlertDialog()
{
    if(operatorAlertDialog == nullptr)
    {
        operatorAlertDialog = new MainDialog();
        operatorAlertDialog->setType(MainDialog::Type::ALERT);
    }

    return *operatorAlertDialog;
}

MainDialog& DialogFactory::getMainConfirmDialog()
{
    if(operatorConfirmDialog == nullptr)
    {
        operatorConfirmDialog = new MainDialog();
        operatorConfirmDialog->setType(MainDialog::Type::CONFIRM);
    }

    return *operatorConfirmDialog;
}

void DialogFactory::showInformation(const QString& message, bool forceModal)
{
    getMainInformationDialog().setText(message);
    if(forceModal == true)
    {
        getMainInformationDialog().exec();
    }
    else
    {
        getMainInformationDialog().show();
    }
}

void DialogFactory::alert(const QString& message, bool forceModal)
{
    getMainAlertDialog().setText(message);
    if(forceModal == true)
    {
        getMainAlertDialog().exec();
    }
    else
    {
        getMainAlertDialog().show();
    }
}

bool DialogFactory::executeConfirm(const QString& message)
{
    getMainConfirmDialog().setText(message);
    return getMainConfirmDialog().exec() == QDialog::Accepted;
}

bool DialogFactory::telecommandCheck(const QString& command)
{
    getMainConfirmDialog().setText(QString("Are you sure want to send the command?\nCommand:  %1").arg(command));
    return getMainConfirmDialog().exec() == QDialog::Accepted;
}

void DialogFactory::telecommandCheck(const QString& command, std::function<void()> const& callbackOk)
{
    auto newDialog = new MainDialog();
    newDialog->setType(MainDialog::Type::CONFIRM);
    newDialog->setAttribute(Qt::WA_DeleteOnClose);
    newDialog->setText(QString("Are you sure want to send the command?\nCommand:  %1").arg(command));
    QObject::connect(newDialog, &QDialog::accepted, callbackOk);
    newDialog->show();
}

bool DialogFactory::telecommandInitCheck()
{
    getMainConfirmDialog().setText("Since the telemetry has not been received, the current status is unknown.\nDo you want to continue processing?");
    return getMainConfirmDialog().exec() == QDialog::Accepted;
}

void DialogFactory::telecommandInitCheck(std::function<void()> const& callbackOk)
{
    auto newDialog = new MainDialog();
    newDialog->setType(MainDialog::Type::CONFIRM);
    newDialog->setAttribute(Qt::WA_DeleteOnClose);
    newDialog->setText("Since the telemetry has not been received, the current status is unknown.\nDo you want to continue processing?");
    QObject::connect(newDialog, &QDialog::accepted, callbackOk);
    newDialog->show();
}

bool DialogFactory::telecommandWaitCheck()
{
    getMainConfirmDialog().setText("Currently waiting for the result of the last command sent.\nDo you want to continue processing?");
    return getMainConfirmDialog().exec() == QDialog::Accepted;
}

void DialogFactory::telecommandWaitCheck(std::function<void()> const& callbackOk)
{
    auto newDialog = new MainDialog();
    newDialog->setType(MainDialog::Type::CONFIRM);
    newDialog->setAttribute(Qt::WA_DeleteOnClose);
    newDialog->setText("Currently waiting for the result of the last command sent.\nDo you want to continue processing?");
    QObject::connect(newDialog, &QDialog::accepted, callbackOk);
    newDialog->show();
}

intball::CameraMicrophoneSettingsDialog& DialogFactory::getCameraMicrophoneSettingsDialog()
{
    if(cameraMicrophoneSettingsDialog == nullptr)
    {
        cameraMicrophoneSettingsDialog = new CameraMicrophoneSettingsDialog();
        cameraMicrophoneSettingsDialog->setStyleSheet(QString("background-color: %1;color: %2;")
                                                      .arg(Color::styleSheetRGB(Color::U1))
                                                      .arg(Color::styleSheetRGB(Color::F1)));
        cameraMicrophoneSettingsDialog->setFont(QFont("Roboto"));
    }
    return *cameraMicrophoneSettingsDialog;
}

intball::LedSettingsDialog& DialogFactory::getLedSettingsDialog()
{
    if(ledSettingsDialog == nullptr)
    {
        ledSettingsDialog = new LedSettingsDialog();
        ledSettingsDialog->setStyleSheet(QString("background-color: %1;color: %2;")
                                         .arg(Color::styleSheetRGB(Color::U1))
                                         .arg(Color::styleSheetRGB(Color::F1)));
        ledSettingsDialog->setFont(QFont("Roboto"));
    }
    return *ledSettingsDialog;
}

void DialogFactory::activate(QDialog* dialog)
{
    if(!dialog->isVisible())
    {
        dialog->show();
    }
    dialog->raise();
    dialog->activateWindow();
}

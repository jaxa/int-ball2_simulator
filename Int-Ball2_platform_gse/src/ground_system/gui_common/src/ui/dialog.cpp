#include <QGraphicsDropShadowEffect>
#include <QIcon>
#include <QPushButton>
#include <QStyle>
#include "dialog.h"
#include "gui_color.h"
#include "ui_dialog.h"

using namespace intball;

MainDialog::MainDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MainDialog),
    type_(Type::INFO)
{
    ui->setupUi(this);
    setWindowFlags(Qt::FramelessWindowHint);
    setAttribute(Qt::WA_NoSystemBackground, true);
    setAttribute(Qt::WA_TranslucentBackground, true);
    setType(Type::INFO);

    connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);

    QGraphicsDropShadowEffect *dropShadow = new QGraphicsDropShadowEffect();
    dropShadow->setBlurRadius(40);
    dropShadow->setColor(QColor(0, 0, 0));
    dropShadow->setOffset(12, 12);
    ui->widget->setGraphicsEffect(dropShadow);
}

MainDialog::~MainDialog()
{
    delete ui;
}

void MainDialog::setText(const QString& message)
{
    ui->label->setText(message);
    if(ui->buttonBox->button(QDialogButtonBox::Cancel))
    {
        // キャンセルボタンが存在する場合は初期選択（ハイライト）する.
        ui->buttonBox->button(QDialogButtonBox::Ok)->setDefault(false);
        ui->buttonBox->button(QDialogButtonBox::Cancel)->setDefault(true);
        ui->buttonBox->button(QDialogButtonBox::Cancel)->setFocus();
    }
}

void MainDialog::setType(const Type& type)
{
    type_ = type;
    QIcon icon;
    switch(type)
    {
    case Type::INFO:
        ui->buttonBox->clear();
        ui->buttonBox->setStandardButtons(QDialogButtonBox::Ok);
        setStyleSheet(QString("* {"
                      "  background-color: %1;"
                      "  color: %2;"
                      "}"
                      "#widget {"
                      "  border: 1px solid %3;"
                      "  border-radius: 6px;"
                      "}")
                      .arg(Color::styleSheetRGB(Color::U1))
                      .arg(Color::styleSheetRGB(Color::F1))
                      .arg(Color::styleSheetRGB(Color::U_HIGHLIGHT)));
        icon = QApplication::style()->standardIcon( QStyle::SP_MessageBoxInformation );
        ui->labelIcon->setPixmap(icon.pixmap(QSize(50, 50)));
        break;
    case Type::ALERT:
        ui->buttonBox->clear();
        ui->buttonBox->setStandardButtons(QDialogButtonBox::Ok);
        setStyleSheet(QString("* {"
                      "  background-color: %1;"
                      "  color: %2;"
                      "}"
                      "#widget {"
                      "  border: 1px solid %3;"
                      "  border-radius: 6px;"
                      "}")
                      .arg(Color::styleSheetRGB(Color::U1))
                      .arg(Color::styleSheetRGB(Color::F1))
                      .arg(Color::styleSheetRGB(Color::S2)));
        icon = QApplication::style()->standardIcon( QStyle::SP_MessageBoxWarning );
        ui->labelIcon->setPixmap(icon.pixmap(QSize(50, 50)));
        break;
    case Type::CONFIRM:
        ui->buttonBox->clear();
        ui->buttonBox->setStandardButtons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        ui->buttonBox->button(QDialogButtonBox::Cancel)->setDefault(true);
        setStyleSheet(QString("* {"
                      "  background-color: %1;"
                      "  color: %2;"
                      "}"
                      "#widget {"
                      "  border: 1px solid %3;"
                      "  border-radius: 6px;"
                      "}")
                      .arg(Color::styleSheetRGB(Color::U1))
                      .arg(Color::styleSheetRGB(Color::F1))
                      .arg(Color::styleSheetRGB(Color::U_HIGHLIGHT)));
        icon = QApplication::style()->standardIcon( QStyle::SP_MessageBoxQuestion );
        ui->labelIcon->setPixmap(icon.pixmap(QSize(50, 50)));
        break;
    }
}

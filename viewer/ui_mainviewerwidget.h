/********************************************************************************
** Form generated from reading UI file 'mainviewerwidget.ui'
**
** Created by: Qt User Interface Compiler version 5.13.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINVIEWERWIDGET_H
#define UI_MAINVIEWERWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainViewerWidget
{
public:
    QWidget *horizontalLayoutWidget;
    QHBoxLayout *main_layout;

    void setupUi(QDialog *MainViewerWidget)
    {
        if (MainViewerWidget->objectName().isEmpty())
            MainViewerWidget->setObjectName(QString::fromUtf8("MainViewerWidget"));
        MainViewerWidget->resize(400, 300);
        horizontalLayoutWidget = new QWidget(MainViewerWidget);
        horizontalLayoutWidget->setObjectName(QString::fromUtf8("horizontalLayoutWidget"));
        horizontalLayoutWidget->setGeometry(QRect(10, 10, 381, 281));
        main_layout = new QHBoxLayout(horizontalLayoutWidget);
        main_layout->setObjectName(QString::fromUtf8("main_layout"));
        main_layout->setContentsMargins(0, 0, 0, 0);

        retranslateUi(MainViewerWidget);

        QMetaObject::connectSlotsByName(MainViewerWidget);
    } // setupUi

    void retranslateUi(QDialog *MainViewerWidget)
    {
        MainViewerWidget->setWindowTitle(QCoreApplication::translate("MainViewerWidget", "Dialog", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainViewerWidget: public Ui_MainViewerWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINVIEWERWIDGET_H

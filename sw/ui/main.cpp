#include <QApplication>
#include <QLabel>
#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QScreen>

int main(int argc, char *argv[])
{
    qputenv("QT_QPA_PLATFORM", QByteArray("linuxfb"));

    QApplication app(argc, argv);

    QWidget* w = new QWidget();
    auto lay = new QVBoxLayout(w);


    auto label = new QLabel("Hello from Qt6 on Raspberry Pi! Piiiip pouuuup");
    lay->addWidget(label);

    auto button = new QPushButton("plop");
    lay->addWidget(button);
    int count = 0;
    QObject::connect(button, &QPushButton::clicked,
		    [=, &count]() {
		   qDebug() << "plop";
		   label->setText(QString::number(count));
		   count += 1;
	});

    w->showFullScreen();

    return app.exec();
}


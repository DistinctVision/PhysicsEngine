#include <QApplication>

#include "QScrollEngine/QScrollEngine.h"
#include "Scene.h"

int main(int argc, char *argv[])
{
    using namespace QScrollEngine;//Все классы движка спрятаны в пространство имен QScrollEngine.
    QApplication app(argc, argv);//Объект приложения Qt.
    app.setApplicationName("Sample");
    QScrollEngineWidget widget;//Создем виджет движка.
    widget.show();//Показываем.
    //Собственно в момент создания происходит инициализация всех ресурсов
    new Scene(&widget);//Создаем нашу сцену.
    //По идее, правильно было хранить ссылку на сцену, чтобы удалить ее в момент выхода из приложения.
    //Но при выходе из приложения удалиться виджет, а вместе с ним и все сцены, связанные с ним. Поэтому хранить ссылку не обязательно.
    return app.exec();
}

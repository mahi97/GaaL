#ifndef PARAMMANAGER_H
#define PARAMMANAGER_H
#include <QSettings>
#include "singleton.hpp"
class ParamManager{
public:
    ParamManager();
    explicit ParamManager(const QString&);
    ~ParamManager();
    bool loadParam(QChar&, const QString&, QChar d = 0);
    bool loadParam(int&, const QString&, int d = 0);
    bool loadParam(double&, const QString&, double d = 0);
    bool loadParam(QString&, const QString&, const QString& d = "");
    bool loadParam(bool&, const QString&, bool d = false);
    bool changeParam(const QString&,const QVariant&);
    bool changeParam(const QString&,const QString&,const QVariant&);
    QStringList allKeys();
    QStringList allKeys(const QString&);
    QStringList allGroups();
    QVariant value(const QString&);
    QVariant value(const QString&,const QString&);
    void sync();
    void clear();
protected:
    QSettings *settings;
};

#endif // PARAMMANAGER_H
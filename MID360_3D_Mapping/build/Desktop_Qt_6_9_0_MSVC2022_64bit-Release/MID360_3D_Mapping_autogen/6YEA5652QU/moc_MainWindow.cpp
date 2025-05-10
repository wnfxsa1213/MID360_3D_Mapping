/****************************************************************************
** Meta object code from reading C++ file 'MainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../include/MainWindow.h"
#include <QtGui/qtextcursor.h>
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN10MainWindowE_t {};
} // unnamed namespace


#ifdef QT_MOC_HAS_STRINGDATA
static constexpr auto qt_meta_stringdata_ZN10MainWindowE = QtMocHelpers::stringData(
    "MainWindow",
    "onStartScan",
    "",
    "onStopScan",
    "onSaveMap",
    "onLoadMap",
    "onClearMap",
    "onUpdatePointCloud",
    "onConfigSettings",
    "onLidarStatusChanged",
    "connected",
    "onLidarError",
    "errorMessage",
    "onProcessFinished",
    "pcl::PointCloud<pcl::PointXYZI>::Ptr",
    "processedCloud",
    "onGlobalMapUpdated",
    "newGlobalMap",
    "onPoseUpdated",
    "Eigen::Matrix4f",
    "pose",
    "onProcessorError"
);
#else  // !QT_MOC_HAS_STRINGDATA
#error "qtmochelpers.h not found or too old."
#endif // !QT_MOC_HAS_STRINGDATA

Q_CONSTINIT static const uint qt_meta_data_ZN10MainWindowE[] = {

 // content:
      12,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   92,    2, 0x08,    1 /* Private */,
       3,    0,   93,    2, 0x08,    2 /* Private */,
       4,    0,   94,    2, 0x08,    3 /* Private */,
       5,    0,   95,    2, 0x08,    4 /* Private */,
       6,    0,   96,    2, 0x08,    5 /* Private */,
       7,    0,   97,    2, 0x08,    6 /* Private */,
       8,    0,   98,    2, 0x08,    7 /* Private */,
       9,    1,   99,    2, 0x08,    8 /* Private */,
      11,    1,  102,    2, 0x08,   10 /* Private */,
      13,    1,  105,    2, 0x08,   12 /* Private */,
      16,    1,  108,    2, 0x08,   14 /* Private */,
      18,    1,  111,    2, 0x08,   16 /* Private */,
      21,    1,  114,    2, 0x08,   18 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   10,
    QMetaType::Void, QMetaType::QString,   12,
    QMetaType::Void, 0x80000000 | 14,   15,
    QMetaType::Void, 0x80000000 | 14,   17,
    QMetaType::Void, 0x80000000 | 19,   20,
    QMetaType::Void, QMetaType::QString,   12,

       0        // eod
};

Q_CONSTINIT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_ZN10MainWindowE.offsetsAndSizes,
    qt_meta_data_ZN10MainWindowE,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_tag_ZN10MainWindowE_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<MainWindow, std::true_type>,
        // method 'onStartScan'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onStopScan'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onSaveMap'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onLoadMap'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onClearMap'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onUpdatePointCloud'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onConfigSettings'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        // method 'onLidarStatusChanged'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<bool, std::false_type>,
        // method 'onLidarError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'onProcessFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::false_type>,
        // method 'onGlobalMapUpdated'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::false_type>,
        // method 'onPoseUpdated'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const Eigen::Matrix4f &, std::false_type>,
        // method 'onProcessorError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>
    >,
    nullptr
} };

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<MainWindow *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->onStartScan(); break;
        case 1: _t->onStopScan(); break;
        case 2: _t->onSaveMap(); break;
        case 3: _t->onLoadMap(); break;
        case 4: _t->onClearMap(); break;
        case 5: _t->onUpdatePointCloud(); break;
        case 6: _t->onConfigSettings(); break;
        case 7: _t->onLidarStatusChanged((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 8: _t->onLidarError((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 9: _t->onProcessFinished((*reinterpret_cast< std::add_pointer_t<pcl::PointCloud<pcl::PointXYZI>::Ptr>>(_a[1]))); break;
        case 10: _t->onGlobalMapUpdated((*reinterpret_cast< std::add_pointer_t<pcl::PointCloud<pcl::PointXYZI>::Ptr>>(_a[1]))); break;
        case 11: _t->onPoseUpdated((*reinterpret_cast< std::add_pointer_t<Eigen::Matrix4f>>(_a[1]))); break;
        case 12: _t->onProcessorError((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ZN10MainWindowE.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 13;
    }
    return _id;
}
QT_WARNING_POP

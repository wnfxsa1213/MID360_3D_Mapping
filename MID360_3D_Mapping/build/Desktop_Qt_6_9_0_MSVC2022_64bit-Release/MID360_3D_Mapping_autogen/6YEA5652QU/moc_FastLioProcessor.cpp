/****************************************************************************
** Meta object code from reading C++ file 'FastLioProcessor.h'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../include/FastLioProcessor.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'FastLioProcessor.h' doesn't include <QObject>."
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
struct qt_meta_tag_ZN16FastLioProcessorE_t {};
} // unnamed namespace


#ifdef QT_MOC_HAS_STRINGDATA
static constexpr auto qt_meta_stringdata_ZN16FastLioProcessorE = QtMocHelpers::stringData(
    "FastLioProcessor",
    "processFinished",
    "",
    "pcl::PointCloud<pcl::PointXYZI>::Ptr",
    "processedCloud",
    "globalMapUpdated",
    "newGlobalMap",
    "poseUpdated",
    "Eigen::Matrix4f",
    "pose",
    "processorError",
    "errorMessage",
    "processPointCloud",
    "cloud",
    "processImuData",
    "ImuData",
    "imuData",
    "processPointCloudWithTimestamp",
    "uint64_t",
    "cloudTimestamp"
);
#else  // !QT_MOC_HAS_STRINGDATA
#error "qtmochelpers.h not found or too old."
#endif // !QT_MOC_HAS_STRINGDATA

Q_CONSTINIT static const uint qt_meta_data_ZN16FastLioProcessorE[] = {

 // content:
      12,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    1,   56,    2, 0x06,    1 /* Public */,
       5,    1,   59,    2, 0x06,    3 /* Public */,
       7,    1,   62,    2, 0x06,    5 /* Public */,
      10,    1,   65,    2, 0x06,    7 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      12,    1,   68,    2, 0x0a,    9 /* Public */,
      14,    1,   71,    2, 0x0a,   11 /* Public */,
      17,    2,   74,    2, 0x0a,   13 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    6,
    QMetaType::Void, 0x80000000 | 8,    9,
    QMetaType::Void, QMetaType::QString,   11,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,   13,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 18,   13,   19,

       0        // eod
};

Q_CONSTINIT const QMetaObject FastLioProcessor::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_ZN16FastLioProcessorE.offsetsAndSizes,
    qt_meta_data_ZN16FastLioProcessorE,
    qt_static_metacall,
    nullptr,
    qt_incomplete_metaTypeArray<qt_meta_tag_ZN16FastLioProcessorE_t,
        // Q_OBJECT / Q_GADGET
        QtPrivate::TypeAndForceComplete<FastLioProcessor, std::true_type>,
        // method 'processFinished'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::false_type>,
        // method 'globalMapUpdated'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::false_type>,
        // method 'poseUpdated'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const Eigen::Matrix4f &, std::false_type>,
        // method 'processorError'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const QString &, std::false_type>,
        // method 'processPointCloud'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::false_type>,
        // method 'processImuData'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<const ImuData &, std::false_type>,
        // method 'processPointCloudWithTimestamp'
        QtPrivate::TypeAndForceComplete<void, std::false_type>,
        QtPrivate::TypeAndForceComplete<pcl::PointCloud<pcl::PointXYZI>::Ptr, std::false_type>,
        QtPrivate::TypeAndForceComplete<uint64_t, std::false_type>
    >,
    nullptr
} };

void FastLioProcessor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<FastLioProcessor *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->processFinished((*reinterpret_cast< std::add_pointer_t<pcl::PointCloud<pcl::PointXYZI>::Ptr>>(_a[1]))); break;
        case 1: _t->globalMapUpdated((*reinterpret_cast< std::add_pointer_t<pcl::PointCloud<pcl::PointXYZI>::Ptr>>(_a[1]))); break;
        case 2: _t->poseUpdated((*reinterpret_cast< std::add_pointer_t<Eigen::Matrix4f>>(_a[1]))); break;
        case 3: _t->processorError((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 4: _t->processPointCloud((*reinterpret_cast< std::add_pointer_t<pcl::PointCloud<pcl::PointXYZI>::Ptr>>(_a[1]))); break;
        case 5: _t->processImuData((*reinterpret_cast< std::add_pointer_t<ImuData>>(_a[1]))); break;
        case 6: _t->processPointCloudWithTimestamp((*reinterpret_cast< std::add_pointer_t<pcl::PointCloud<pcl::PointXYZI>::Ptr>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<uint64_t>>(_a[2]))); break;
        default: ;
        }
    }
    if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _q_method_type = void (FastLioProcessor::*)(pcl::PointCloud<pcl::PointXYZI>::Ptr );
            if (_q_method_type _q_method = &FastLioProcessor::processFinished; *reinterpret_cast<_q_method_type *>(_a[1]) == _q_method) {
                *result = 0;
                return;
            }
        }
        {
            using _q_method_type = void (FastLioProcessor::*)(pcl::PointCloud<pcl::PointXYZI>::Ptr );
            if (_q_method_type _q_method = &FastLioProcessor::globalMapUpdated; *reinterpret_cast<_q_method_type *>(_a[1]) == _q_method) {
                *result = 1;
                return;
            }
        }
        {
            using _q_method_type = void (FastLioProcessor::*)(const Eigen::Matrix4f & );
            if (_q_method_type _q_method = &FastLioProcessor::poseUpdated; *reinterpret_cast<_q_method_type *>(_a[1]) == _q_method) {
                *result = 2;
                return;
            }
        }
        {
            using _q_method_type = void (FastLioProcessor::*)(const QString & );
            if (_q_method_type _q_method = &FastLioProcessor::processorError; *reinterpret_cast<_q_method_type *>(_a[1]) == _q_method) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject *FastLioProcessor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *FastLioProcessor::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ZN16FastLioProcessorE.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int FastLioProcessor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void FastLioProcessor::processFinished(pcl::PointCloud<pcl::PointXYZI>::Ptr _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void FastLioProcessor::globalMapUpdated(pcl::PointCloud<pcl::PointXYZI>::Ptr _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void FastLioProcessor::poseUpdated(const Eigen::Matrix4f & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void FastLioProcessor::processorError(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP

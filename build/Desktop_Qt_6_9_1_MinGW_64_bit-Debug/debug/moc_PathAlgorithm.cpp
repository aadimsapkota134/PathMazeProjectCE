/****************************************************************************
** Meta object code from reading C++ file 'PathAlgorithm.h'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.9.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../headers/PathAlgorithm.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PathAlgorithm.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 69
#error "This file was generated using the moc from 6.9.1. It"
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
struct qt_meta_tag_ZN13PathAlgorithmE_t {};
} // unnamed namespace

template <> constexpr inline auto PathAlgorithm::qt_create_metaobjectdata<qt_meta_tag_ZN13PathAlgorithmE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "PathAlgorithm",
        "updatedScatterGridView",
        "",
        "UPDATETYPES",
        "VISIT",
        "currentIndex",
        "updatedLineGridView",
        "currentPoint",
        "addingPoint",
        "clearPriorToUpdate",
        "algorithmCompleted",
        "pathfindingSearchCompleted",
        "nodesVisited",
        "pathLength"
    };

    QtMocHelpers::UintData qt_methods {
        // Signal 'updatedScatterGridView'
        QtMocHelpers::SignalData<void(UPDATETYPES, int)>(1, 2, QMC::AccessPublic, QMetaType::Void, {{
            { 0x80000000 | 3, 4 }, { QMetaType::Int, 5 },
        }}),
        // Signal 'updatedLineGridView'
        QtMocHelpers::SignalData<void(QPointF, bool, bool)>(6, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::QPointF, 7 }, { QMetaType::Bool, 8 }, { QMetaType::Bool, 9 },
        }}),
        // Signal 'updatedLineGridView'
        QtMocHelpers::SignalData<void(QPointF, bool)>(6, 2, QMC::AccessPublic | QMC::MethodCloned, QMetaType::Void, {{
            { QMetaType::QPointF, 7 }, { QMetaType::Bool, 8 },
        }}),
        // Signal 'algorithmCompleted'
        QtMocHelpers::SignalData<void()>(10, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'pathfindingSearchCompleted'
        QtMocHelpers::SignalData<void(int, int)>(11, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Int, 12 }, { QMetaType::Int, 13 },
        }}),
    };
    QtMocHelpers::UintData qt_properties {
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<PathAlgorithm, qt_meta_tag_ZN13PathAlgorithmE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject PathAlgorithm::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN13PathAlgorithmE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN13PathAlgorithmE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN13PathAlgorithmE_t>.metaTypes,
    nullptr
} };

void PathAlgorithm::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<PathAlgorithm *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->updatedScatterGridView((*reinterpret_cast< std::add_pointer_t<UPDATETYPES>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        case 1: _t->updatedLineGridView((*reinterpret_cast< std::add_pointer_t<QPointF>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[3]))); break;
        case 2: _t->updatedLineGridView((*reinterpret_cast< std::add_pointer_t<QPointF>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2]))); break;
        case 3: _t->algorithmCompleted(); break;
        case 4: _t->pathfindingSearchCompleted((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        default: ;
        }
    }
    if (_c == QMetaObject::IndexOfMethod) {
        if (QtMocHelpers::indexOfMethod<void (PathAlgorithm::*)(UPDATETYPES , int )>(_a, &PathAlgorithm::updatedScatterGridView, 0))
            return;
        if (QtMocHelpers::indexOfMethod<void (PathAlgorithm::*)(QPointF , bool , bool )>(_a, &PathAlgorithm::updatedLineGridView, 1))
            return;
        if (QtMocHelpers::indexOfMethod<void (PathAlgorithm::*)()>(_a, &PathAlgorithm::algorithmCompleted, 3))
            return;
        if (QtMocHelpers::indexOfMethod<void (PathAlgorithm::*)(int , int )>(_a, &PathAlgorithm::pathfindingSearchCompleted, 4))
            return;
    }
}

const QMetaObject *PathAlgorithm::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PathAlgorithm::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN13PathAlgorithmE_t>.strings))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int PathAlgorithm::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void PathAlgorithm::updatedScatterGridView(UPDATETYPES _t1, int _t2)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 0, nullptr, _t1, _t2);
}

// SIGNAL 1
void PathAlgorithm::updatedLineGridView(QPointF _t1, bool _t2, bool _t3)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 1, nullptr, _t1, _t2, _t3);
}

// SIGNAL 3
void PathAlgorithm::algorithmCompleted()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void PathAlgorithm::pathfindingSearchCompleted(int _t1, int _t2)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 4, nullptr, _t1, _t2);
}
QT_WARNING_POP

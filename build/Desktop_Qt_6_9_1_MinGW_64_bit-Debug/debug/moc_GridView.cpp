/****************************************************************************
** Meta object code from reading C++ file 'GridView.h'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.9.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../headers/GridView.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GridView.h' doesn't include <QObject>."
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
struct qt_meta_tag_ZN8GridViewE_t {};
} // unnamed namespace

template <> constexpr inline auto GridView::qt_create_metaobjectdata<qt_meta_tag_ZN8GridViewE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "GridView",
        "handleClickedPoint",
        "",
        "point",
        "handleUpdatedScatterGridView",
        "UPDATETYPES",
        "updateType",
        "updateIndex",
        "handleUpdatedLineGridView",
        "updatePoint",
        "addingPoint",
        "clearPriorToUpdate"
    };

    QtMocHelpers::UintData qt_methods {
        // Slot 'handleClickedPoint'
        QtMocHelpers::SlotData<void(const QPointF &)>(1, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::QPointF, 3 },
        }}),
        // Slot 'handleUpdatedScatterGridView'
        QtMocHelpers::SlotData<bool(UPDATETYPES, int)>(4, 2, QMC::AccessPublic, QMetaType::Bool, {{
            { 0x80000000 | 5, 6 }, { QMetaType::Int, 7 },
        }}),
        // Slot 'handleUpdatedLineGridView'
        QtMocHelpers::SlotData<bool(QPointF, bool, bool)>(8, 2, QMC::AccessPublic, QMetaType::Bool, {{
            { QMetaType::QPointF, 9 }, { QMetaType::Bool, 10 }, { QMetaType::Bool, 11 },
        }}),
        // Slot 'handleUpdatedLineGridView'
        QtMocHelpers::SlotData<bool(QPointF, bool)>(8, 2, QMC::AccessPublic | QMC::MethodCloned, QMetaType::Bool, {{
            { QMetaType::QPointF, 9 }, { QMetaType::Bool, 10 },
        }}),
    };
    QtMocHelpers::UintData qt_properties {
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<GridView, qt_meta_tag_ZN8GridViewE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject GridView::staticMetaObject = { {
    QMetaObject::SuperData::link<QChartView::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN8GridViewE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN8GridViewE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN8GridViewE_t>.metaTypes,
    nullptr
} };

void GridView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<GridView *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->handleClickedPoint((*reinterpret_cast< std::add_pointer_t<QPointF>>(_a[1]))); break;
        case 1: { bool _r = _t->handleUpdatedScatterGridView((*reinterpret_cast< std::add_pointer_t<UPDATETYPES>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 2: { bool _r = _t->handleUpdatedLineGridView((*reinterpret_cast< std::add_pointer_t<QPointF>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[3])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 3: { bool _r = _t->handleUpdatedLineGridView((*reinterpret_cast< std::add_pointer_t<QPointF>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<bool>>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
}

const QMetaObject *GridView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GridView::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN8GridViewE_t>.strings))
        return static_cast<void*>(this);
    return QChartView::qt_metacast(_clname);
}

int GridView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QChartView::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 4;
    }
    return _id;
}
QT_WARNING_POP

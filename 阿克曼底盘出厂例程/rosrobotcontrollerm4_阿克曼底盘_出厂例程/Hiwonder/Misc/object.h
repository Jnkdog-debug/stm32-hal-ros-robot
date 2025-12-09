#ifndef __OBJECT_H
#define __OBJECT_H

typedef enum {
	OBJECT_TYPE_ID_TYPE = 0x00,
	OBJECT_TYPE_ID_KEY_EVENT = 0x01,
	OBJECT_TYPE_ID_SBUS_STATUS,
	OBJECT_TYPE_ID_GAMEPAD_STATUS,
}ObjectTypeIDEnum;

/**
 * @brief 对象基类结构体
 * @details 所有事件对象的基类，包含对象类型信息
 */
typedef struct {
	ObjectTypeIDEnum type_id;  /**< @brief 对象类型ID */
} ObjectTypeDef;

#endif



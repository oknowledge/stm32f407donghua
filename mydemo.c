#include "mydemo.h"
LV_IMG_DECLARE(bird3) 

// �����ص����������ڸ���x���y���ֵ�Լ�ͼƬ��С
static void anim_x_y_size_cb(void* var, int32_t v) 
{
    lv_obj_t* img = (lv_obj_t*)var;
    // ����x���y����x���y���ֵ�����ݶ������ȴ����½��ƶ������Ͻ�
    int32_t x = 480 - (v * 480 / 5000);  // ���趯��ʱ��Ϊ5000ms
    int32_t y = 320 - (v * 320 / 5000);
    lv_obj_set_x(img, x);
    lv_obj_set_y(img, y);

    // ����ͼƬ��С�����ݶ���������С���
    int32_t size = 50 + (v * 180 / 5000);  // ������ʼ��СΪ50�����մ�СΪ150
    lv_obj_set_width(img, size);
    lv_obj_set_height(img, size);
}

void lv_example(void) 
{
    /* ��һ��������ͼƬ���� */ 
    lv_obj_t* lv_obj = lv_img_create(lv_scr_act());
    lv_img_set_src(lv_obj, &bird3);

    /* �ڶ�����������ʼ�� */ 
    lv_anim_t a;
    lv_anim_init(&a);

    /* �����������ö���Ŀ��ΪͼƬ���� */ 
    lv_anim_set_var(&a, lv_obj);

    /* ���Ĳ������ö��������յ� */ 
    // ����ʹ��һ���ϴ����ֵ��Ϊ�������ȵķ�Χ����Ϊ�����ڻص������и��������Χ����ʵ�ʵ�λ�úʹ�С�仯
    lv_anim_set_values(&a, 0, 5000); 

    /* ���岽�����ö���ʱ�� */ 
    lv_anim_set_time(&a, 5000);

    /* �����������ö����ص����� */ 
    lv_anim_set_exec_cb(&a, anim_x_y_size_cb);

    /* ���߲������ö������ */ 
    lv_anim_set_path_cb(&a, lv_anim_path_overshoot);

    /* �ڰ˲����������� */ 
    lv_anim_start(&a); 
} 
#include <aide.h>

void score();

#define BTN_NUMBER 6
#define BTN_WIDTH 55
#define BTN_HEIGHT 55

static void event_btn(lv_event_t * e);
static lv_style_t btn_strat_orginal;
static lv_style_t btn_strat_symetrique;

void Style_btn_confirm();
static lv_style_t style;
static lv_style_t style_pr;
static lv_style_transition_dsc_t trans;
static lv_style_prop_t props[] = {LV_STYLE_OUTLINE_WIDTH, LV_STYLE_OUTLINE_OPA, 0};
int confirm = 0;

lv_obj_t * table;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;

void strategie()
{
	Style_btn_confirm();
	lv_style_init(&btn_strat_orginal);
	lv_style_set_bg_color(&btn_strat_orginal, lv_palette_main(LV_PALETTE_BLUE));
	lv_style_set_radius(&btn_strat_orginal, 80);
	lv_style_set_width(&btn_strat_orginal, BTN_WIDTH);
	lv_style_set_height(&btn_strat_orginal, BTN_HEIGHT);

	lv_style_init(&btn_strat_symetrique);
	lv_style_set_bg_color(&btn_strat_symetrique, lv_palette_main(LV_PALETTE_AMBER));
	lv_style_set_radius(&btn_strat_symetrique, 80);
	lv_style_set_width(&btn_strat_symetrique, BTN_WIDTH);
	lv_style_set_height(&btn_strat_symetrique, BTN_HEIGHT);

	LV_IMG_DECLARE(TableRedi);
	table = lv_img_create(lv_scr_act());
	lv_obj_set_size(table, 480, 272);
	lv_obj_center(table);
	lv_img_set_src(table, &TableRedi);

	uint32_t i;
	for(i = 0; i < BTN_NUMBER; i++) {
		lv_obj_t * btn = lv_btn_create(table);
		lv_obj_add_flag(btn, LV_OBJ_FLAG_EVENT_BUBBLE);
		switch(i)
		{
		case 0:
			lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 423-(BTN_WIDTH/2), 30-(BTN_HEIGHT/2));
			lv_obj_add_style(btn, &btn_strat_orginal, 0);
			break;
		case 1:
			lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 61-(BTN_WIDTH/2), 30-(BTN_HEIGHT/2));
			lv_obj_add_style(btn, &btn_strat_symetrique, 0);
			break;
		case 2:
			lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 20-(BTN_WIDTH/2), 150-(BTN_HEIGHT/2));
			lv_obj_add_style(btn, &btn_strat_orginal, 0);
			break;
		case 3:
			lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 440-(BTN_WIDTH/2), 150-(BTN_HEIGHT/2));
			lv_obj_add_style(btn, &btn_strat_symetrique, 0);
			break;
		case 4:
			lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 285-(BTN_WIDTH/2), 241-(BTN_HEIGHT/2));
			lv_obj_add_style(btn, &btn_strat_orginal, 0);
			break;
		case 5:
			lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 190-(BTN_WIDTH/2), 241-(BTN_HEIGHT/2));
			lv_obj_add_style(btn, &btn_strat_symetrique, 0);
			break;
		default:
			break;
		}

		lv_obj_t * label = lv_label_create(btn);
		lv_label_set_text_fmt(label, "%"LV_PRIu32, i);
		lv_obj_center(label);
	}

	lv_obj_t * btn1 = lv_btn_create(table);
	lv_obj_remove_style_all(btn1);
	lv_obj_add_style(btn1, &style, 0);
	lv_obj_add_style(btn1, &style_pr, LV_STATE_PRESSED);
	lv_obj_set_size(btn1, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
	lv_obj_center(btn1);
	lv_obj_add_flag(btn1, LV_OBJ_FLAG_EVENT_BUBBLE);

	lv_obj_t * lab = lv_label_create(btn1);
	lv_label_set_text(lab, "Confirm");
	lv_obj_center(lab);

	lv_obj_add_event_cb(table, event_btn, LV_EVENT_CLICKED, NULL);
}

void Style_btn_confirm()
{
	lv_style_init(&style);

	lv_style_set_radius(&style, 3);

	lv_style_set_bg_opa(&style, LV_OPA_100);
	lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_DEEP_PURPLE));
	lv_style_set_bg_grad_color(&style, lv_palette_main(LV_PALETTE_PURPLE));
	lv_style_set_bg_grad_dir(&style, LV_GRAD_DIR_VER);

	lv_style_set_border_opa(&style, LV_OPA_40);
	lv_style_set_border_width(&style, 2);
	lv_style_set_border_color(&style, lv_palette_main(LV_PALETTE_PINK));

	lv_style_set_shadow_width(&style, 8);
	lv_style_set_shadow_color(&style, lv_palette_main(LV_PALETTE_GREY));
	lv_style_set_shadow_ofs_y(&style, 8);

	lv_style_set_outline_opa(&style, LV_OPA_COVER);
	lv_style_set_outline_color(&style, lv_palette_main(LV_PALETTE_YELLOW));

	lv_style_set_text_color(&style, lv_color_white());
	lv_style_set_pad_all(&style, 10);

	lv_style_init(&style_pr);

	lv_style_set_outline_width(&style_pr, 30);
	lv_style_set_outline_opa(&style_pr, LV_OPA_TRANSP);

	lv_style_set_translate_y(&style_pr, 5);
	lv_style_set_shadow_ofs_y(&style_pr, 3);
	lv_style_set_bg_color(&style_pr, lv_palette_main(LV_PALETTE_ORANGE));
	lv_style_set_bg_grad_color(&style_pr, lv_palette_main(LV_PALETTE_AMBER));

	lv_style_transition_dsc_init(&trans, props, lv_anim_path_linear, 300, 0, NULL);

	lv_style_set_transition(&style_pr, &trans);
}

static void event_btn(lv_event_t * e)
{
	lv_obj_t * target = lv_event_get_target(e);
	lv_obj_t * cont = lv_event_get_current_target(e);
	if (target == cont) return;

	int Index = lv_obj_get_index(target);
	if (confirm == 0 && Index == BTN_NUMBER) // Bouton "Confirm"
	{
		confirm++;
		lv_color_t r = lv_palette_main(LV_PALETTE_PINK);
		for (int i = 0; i < BTN_NUMBER; i++)
		{
			lv_obj_t * obj = lv_obj_get_child(cont, i);
			lv_color_t color = lv_obj_get_style_bg_color(obj, LV_PART_MAIN);
			if (color.full == r.full)
			{
				char command[8];
				int taille = sprintf(command, "file%d\r\n", i);
				if (taille > 0)
				{
					HAL_UART_Transmit(&huart6, (unsigned char *)command, taille, 100);
					HAL_UART_Transmit(&huart1, (unsigned char *)command, taille, 100);
				}
			}
			// Réinitialisation des couleurs
			lv_obj_set_style_bg_color(obj, (i % 2 == 0) ? lv_palette_main(LV_PALETTE_AMBER) : lv_palette_main(LV_PALETTE_BLUE), 0);
		}
		lv_obj_del(table); // Suppression immédiate
		score(); // Appel direct à score()
	}
	else if (confirm == 0 && Index != BTN_NUMBER) // Bouton numérique
	{
		for (int i = 0; i < BTN_NUMBER; i++)
		{
			lv_obj_t * obj = lv_obj_get_child(cont, i);
			lv_obj_set_style_bg_color(obj, (obj == target) ? lv_palette_main(LV_PALETTE_PINK) :
					(i % 2 == 0) ? lv_palette_main(LV_PALETTE_AMBER) : lv_palette_main(LV_PALETTE_BLUE), 0);
		}
	}
}

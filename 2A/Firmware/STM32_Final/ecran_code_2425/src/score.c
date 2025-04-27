#include <aide.h>

static lv_obj_t * chiffre0;
static lv_obj_t * chiffre1;
static lv_obj_t * chiffre2;

LV_IMG_DECLARE(n0);
LV_IMG_DECLARE(n1);
LV_IMG_DECLARE(n2);
LV_IMG_DECLARE(n3);
LV_IMG_DECLARE(n4);
LV_IMG_DECLARE(n5);
LV_IMG_DECLARE(n6);
LV_IMG_DECLARE(n7);
LV_IMG_DECLARE(n8);
LV_IMG_DECLARE(n9);

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
extern uint8_t pData[5];
extern uint8_t Tab[3];

void score()
{
    lv_obj_t * panel1 = lv_obj_create(lv_scr_act());
    lv_obj_set_height(panel1, LV_SIZE_CONTENT);
    lv_obj_align(panel1, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(panel1, 480, 272);

    lv_obj_clear_flag(panel1, LV_OBJ_FLAG_SCROLLABLE); // Permet de rendre l'objet non défilable
    lv_obj_set_scrollbar_mode(panel1, LV_SCROLLBAR_MODE_OFF); // Permet de retirer les scrollbar

    chiffre0 = lv_img_create(panel1);
    chiffre1 = lv_img_create(panel1);
    chiffre2 = lv_img_create(panel1);

    lv_img_set_src(chiffre0, &n0);
    lv_img_set_src(chiffre1, &n0);
    lv_img_set_src(chiffre2, &n0);

    static lv_coord_t grid_col_dsc[] = {150, 150, 150, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t grid_row_dsc[] = {260, LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(panel1, grid_col_dsc, grid_row_dsc);

    lv_obj_set_grid_cell(chiffre0, LV_GRID_ALIGN_START, 0, 1, LV_GRID_ALIGN_CENTER, 0, 1);
    lv_obj_set_grid_cell(chiffre1, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_CENTER, 0, 1);
    lv_obj_set_grid_cell(chiffre2, LV_GRID_ALIGN_START, 2, 1, LV_GRID_ALIGN_CENTER, 0, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6) {
        // Relancer la réception pour la prochaine interruption
        HAL_UART_Receive_IT(&huart6, pData, 4);

        // Transmettre les données reçues
        HAL_UART_Transmit(&huart1, pData, 4, HAL_MAX_DELAY);

        // Convertir les caractères en entiers
        int score[3] = {Tab[0] - '0', Tab[1] - '0', Tab[2] - '0'};
        int received[3] = {pData[1] - '0', pData[2] - '0', pData[3] - '0'};

        // Vérifier que les données reçues sont valides (chiffres de 0 à 9)
        for (int i = 0; i < 3; i++) {
            if (received[i] < 0 || received[i] > 9) received[i] = 0;
        }

        // Effectuer l’opération
        if (pData[0] == '0') { // Soustraction
            for (int i = 2; i >= 0; i--) {
                score[i] -= received[i];
                if (score[i] < 0 && i > 0) {
                    score[i] += 10;    // Emprunt
                    score[i - 1] -= 1;
                }
            }
        } else { // Addition
            for (int i = 2; i >= 0; i--) {
                score[i] += received[i];
                if (score[i] >= 10 && i > 0) {
                    score[i] -= 10;    // Retenue
                    score[i - 1] += 1;
                }
            }
        }

        // Limiter les valeurs entre 0 et 9 et mettre à jour Tab
        for (int i = 0; i < 3; i++) {
            if (score[i] < 0) score[i] = 0;
            if (score[i] > 9) score[i] = 9;
            Tab[i] = score[i] + '0';
        }

        // Mettre à jour les images directement
        switch (Tab[0]) {
            case '0': lv_img_set_src(chiffre0, &n0); break;
            case '1': lv_img_set_src(chiffre0, &n1); break;
            case '2': lv_img_set_src(chiffre0, &n2); break;
            case '3': lv_img_set_src(chiffre0, &n3); break;
            case '4': lv_img_set_src(chiffre0, &n4); break;
            case '5': lv_img_set_src(chiffre0, &n5); break;
            case '6': lv_img_set_src(chiffre0, &n6); break;
            case '7': lv_img_set_src(chiffre0, &n7); break;
            case '8': lv_img_set_src(chiffre0, &n8); break;
            case '9': lv_img_set_src(chiffre0, &n9); break;
            default:  lv_img_set_src(chiffre0, &n0); break;
        }
        switch (Tab[1]) {
            case '0': lv_img_set_src(chiffre1, &n0); break;
            case '1': lv_img_set_src(chiffre1, &n1); break;
            case '2': lv_img_set_src(chiffre1, &n2); break;
            case '3': lv_img_set_src(chiffre1, &n3); break;
            case '4': lv_img_set_src(chiffre1, &n4); break;
            case '5': lv_img_set_src(chiffre1, &n5); break;
            case '6': lv_img_set_src(chiffre1, &n6); break;
            case '7': lv_img_set_src(chiffre1, &n7); break;
            case '8': lv_img_set_src(chiffre1, &n8); break;
            case '9': lv_img_set_src(chiffre1, &n9); break;
            default:  lv_img_set_src(chiffre1, &n0); break;
        }
        switch (Tab[2]) {
            case '0': lv_img_set_src(chiffre2, &n0); break;
            case '1': lv_img_set_src(chiffre2, &n1); break;
            case '2': lv_img_set_src(chiffre2, &n2); break;
            case '3': lv_img_set_src(chiffre2, &n3); break;
            case '4': lv_img_set_src(chiffre2, &n4); break;
            case '5': lv_img_set_src(chiffre2, &n5); break;
            case '6': lv_img_set_src(chiffre2, &n6); break;
            case '7': lv_img_set_src(chiffre2, &n7); break;
            case '8': lv_img_set_src(chiffre2, &n8); break;
            case '9': lv_img_set_src(chiffre2, &n9); break;
            default:  lv_img_set_src(chiffre2, &n0); break;
        }
    }
}

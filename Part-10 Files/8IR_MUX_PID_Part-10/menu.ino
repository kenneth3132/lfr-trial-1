void menu() {
  while (1) {
    if (current_screen == 0) {  // MENU SCREEN

      // up and down buttons only work for the menu screen
      if ((digitalRead(BUTTON_UP_PIN) == LOW) && (button_up_clicked == 0)) {  // up button clicked - jump to previous menu item
        item_selected = item_selected - 1;                                    // select previous item
        button_up_clicked = 1;                                                // set button to clicked to only perform the action once
        if (item_selected < 0) {                                              // if first item was selected, jump to last item
          item_selected = NUM_ITEMS - 1;
        }
      } else if ((digitalRead(BUTTON_DOWN_PIN) == LOW) && (button_down_clicked == 0)) {  // down button clicked - jump to next menu item
        item_selected = item_selected + 1;                                               // select next item
        button_down_clicked = 1;                                                         // set button to clicked to only perform the action once
        if (item_selected >= NUM_ITEMS) {                                                // last item was selected, jump to first menu item
          item_selected = 0;
        }
      }

      if ((digitalRead(BUTTON_UP_PIN) == HIGH) && (button_up_clicked == 1)) {  // unclick
        button_up_clicked = 0;
      }
      if ((digitalRead(BUTTON_DOWN_PIN) == HIGH) && (button_down_clicked == 1)) {  // unclick
        button_down_clicked = 0;
      }
    }


    if ((digitalRead(BUTTON_SELECT_PIN) == LOW) && (button_select_clicked == 0)) {  // select button clicked, jump between screens
      button_select_clicked = 1;                                                    // set button to clicked to only perform the action once
      if (current_screen == 0) { current_screen = 1; }                              // menu items screen --> screenshots screen
      else if (current_screen == 1) {
        current_screen = 2;
      }                             // screenshots screen --> qr codes screen
      else { current_screen = 0; }  // qr codes screen --> menu items screen
    }
    if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (button_select_clicked == 1)) {  // unclick
      button_select_clicked = 0;
    }

    // set correct values for the previous and next items
    item_sel_previous = item_selected - 1;
    if (item_sel_previous < 0) { item_sel_previous = NUM_ITEMS - 1; }  // previous item would be below first = make it the last
    item_sel_next = item_selected + 1;
    if (item_sel_next >= NUM_ITEMS) { item_sel_next = 0; }  // next item would be after last = make it the first

    //execute specific task when select button is clicked
    if (item_selected == 0 && button_select_clicked == 1) {
      current_screen = 0;
      PID_Controller(base_speed, kp, kd);
    }
    if (item_selected == 1 && button_select_clicked == 1) {
      current_screen = 0;
      calibrateSensor();
    }
    if (item_selected == 2 && button_select_clicked == 1) {
      current_screen = 0;
      analog_value();
    }
    if (item_selected == 3 && button_select_clicked == 1) {
      current_screen = 0;
      digital_value();
    }
    if (item_selected == 4 && button_select_clicked == 1) {
      current_screen = 0;
      motor_test();
    }



    u8g.firstPage();  // required for page drawing mode for u8g library
    do {

      if (current_screen == 0) {  // MENU SCREEN

        // selected item background
        u8g.drawBitmapP(0, 22, 128 / 8, 21, bitmap_item_sel_outline);

        // draw previous item as icon + label
        u8g.setFont(u8g_font_7x14);
        u8g.drawStr(25, 15, menu_items[item_sel_previous]);
        //u8g.drawBitmapP( 4, 2, 16/8, 16, bitmap_icons[item_sel_previous]);

        // draw selected item as icon + label in bold font
        u8g.setFont(u8g_font_7x14B);
        u8g.drawStr(25, 15 + 20 + 2, menu_items[item_selected]);
        //u8g.drawBitmapP( 4, 24, 16/8, 16, bitmap_icons[item_selected]);

        // draw next item as icon + label
        u8g.setFont(u8g_font_7x14);
        u8g.drawStr(25, 15 + 20 + 20 + 2 + 2, menu_items[item_sel_next]);
        //u8g.drawBitmapP( 4, 46, 16/8, 16, bitmap_icons[item_sel_next]);

        // draw scrollbar background
        u8g.drawBitmapP(128 - 8, 0, 8 / 8, 64, bitmap_scrollbar_background);

        // draw scrollbar handle
        u8g.drawBox(125, 64 / NUM_ITEMS * item_selected, 3, 64 / NUM_ITEMS);

        // draw upir logo
        u8g.drawBitmapP(128 - 16 - 4, 64 - 4, 16 / 8, 4, upir_logo);
      }
    } while (u8g.nextPage());  // required for page drawing mode with u8g library
  }
}
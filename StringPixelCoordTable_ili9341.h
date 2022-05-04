// String and Pixel Coord Table for ILI9341


// there is 5px padding around the outer edge of the Display for placing elements,
// due to viewing Angle issues


// Please Select Font for HE-Level Display 
//#define inconsolata
//#define Lucida_Console
#define Lucida_Console_Alpha_Num
//#define source_code_pro
//#define consolas


//BORDER_OFFSET:
/////////////////////////////
//           /\            //
//           ||            //
//           5px           //
//           ||            //
//           \/            //
//<--5px-->content<--5px-->//
//           /\            //
//           ||            //
//           5px           //
//           ||            //
//           \/            //
/////////////////////////////
#define BORDER_OFFSET 5
#define LCD_WIDTH  319
#define LCD_HEIGHT 239

#define FONT1_H 16
#define FONT1_W 8

#define FONT2_H 24
#define FONT2_W 12

// High res Fonts
#ifdef Lucida_Console_Alpha_Num
#define FONT3_H 40
#define FONT3_W 34
#define FONT3_W1 29
#endif

#ifdef Lucida_Console
#define FONT3_H 40
#define FONT3_W 27
#endif

#ifdef inconsolata
#define FONT3_H 40
#define FONT3_W 27
#endif

#ifdef consolas
#define FONT3_H 40
#define FONT3_W 30
#endif

#ifdef source_code_pro
#define FONT3_H 40
#define FONT3_W 31
#endif


//======================================================================================
//MAIN SCREEN
//======================================================================================
#define X_LEFT_EDGE (BORDER_OFFSET+3) 
#define Y_VALUES_START 60
#define DESCRUPTOR_LEN 4





#define CENTER_LINE (LCD_WIDTH-XOFFSET_32)/2

//Font Params //TODO use Values that are defined by the Font
#define CHAR_CELL_WIDTH_FONT_1 8//6
#define HALF_SPACE_WIDTH_FONT_1 4
#define CHAR_CELL_WIDTH_FONT_2 12//9
#define HALF_SPACE_WIDTH_FONT_2 6//3

//InitScreen_AddLine()
#define IAdd_Line_Max_Lines 15//12
#define IAdd_Line_LineFeed  16//10
#define X_IA_2 X_LEFT_EDGE



//======================================================================================
//paint_info_line()
//======================================================================================
#define X_PIL_2 20		 // Info line Directly above Progress-Bar and to the right of
#define Y_PIL_90 50 // Buttons/edge


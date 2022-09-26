#pragma once

#include <memory>
#include <vector>
#include <fstream>
#include <string>

#include "railos/utilities.hxx"
#include "railos/track.hxx"
#include "railos/logging.hxx"

namespace RailOS::Text {
    class TextItem {
        public:
        std::string text_string{""};
        int h_pos{-1}, v_pos{-1};
        //TFont* font;
        TextItem();
        //TextItem(int horizontal, int vertical, const std::string& text, TFont *&input_font);
    };

    class TextHandler {
        public:
            typedef std::vector<TextItem> TextVector;
            typedef std::vector<TextItem>::iterator TextVectorIterator;
            //typedef std::vector<TFont*> TFontVector;
            //TFontVector font_vector;
            TextVector text_vector, select_text_vector;

            void setNewHPos(int caller, int text_item, int new_h_pos) {
                textPtrAt(24, text_item)->h_pos = new_h_pos;
            }

            std::shared_ptr<TextItem> textPtrAt(int caller, int at);

            ~TextHandler() {
                text_vector.clear();
            }

            bool checkTextElementsInFile(int caller, std::ifstream& vec_file);
            ///< check the validity of text items in VecFile prior to loading, reutnr true for success

            bool findText(int caller, const std::string& name, int& h_pos, int& v_pos);
            ///< look in TextVector for text item 'Name' and if fourn return true and return its position

            //bool fontSame(int caller, TFont* existing, TFont* input_font);

            bool textFound(int caller, int h_pos_input, int v_pos_input, const std::string& text);
            ///< look for a text item in the vicinity of h_pos_input and v_pos_input

            //int getFontStyleAsInt(int caller, TFont* input_font);

            std::shared_ptr<TextItem> selectTextPtrAt(int caller, int at);
            ///< return the text item at position 'At' in SelectTextVector (carries out range checking)

            unsigned int selectTextVectorSize(int caller);
            ///< return the number of items in SelectTextVector

            unsigned int textVectorSize(int caller);
            ///< return the number of items in TextVector

            void enterAndDisplayNewText(int caller, TextItem text, int h_pos, int v_pos);
            ///< add Text to TextVector and display it on the screen

            void loadText(int caller, std::ifstream& vec_file);
            ///< load the railway's text from VecFile

            //void RebuildFromTextVector(int Caller, TDisplay *Disp);
            ///< display all text items in TextVector on the screen
            
            void saveText(int caller, std::ofstream& vec_file);
            ///< save the railway's text to VecFile

            void textClear(int caller);
            ///< empties TextVector and sets all offsets back to zero if there is no active or inactive track

            void textMove(int caller, int h_pos_input, int v_pos_input, int &text_item, int &text_move_h_pos, int &text_move_v_pos, bool &text_found_flag);
            ///< look for a text item in the vicinity of HPosInput & VPosInput & if found return its exact position in &TextMoveHPos & &TextMoveVPos, its
            ///< position in TextVector in TextItem, and set FoundFlag to true

            void textVectorPush(int caller, TextItem text);
            ///< push &Text onto TextVector & reset the size of the railway if necessary

            void textVectorResetPosition(int caller, int h_offset, int v_offset);
            ///< change the HPos & VPos values for all items in TextVector by the amount in HOffset and VOffset (unused)

            //void writeTextToImage(int caller, Graphics::TBitmap *Bitmap);
            ///< write all items in TextVector to the railway image in 'Bitmap'
    };

    extern std::shared_ptr<TextHandler> text_handler; // the object pointer, object created in InterfaceUnit
};

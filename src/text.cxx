#include "railos/text.hxx"
#include "railos/logging.hxx"
#include <string>

namespace RailOS::Text {
    TextItem::TextItem() : text_string(""), h_pos(0), v_pos(0) {}
    void TextHandler::textMove(int caller, int h_Pos_input, int v_pos_input, int &tedxt_item, int &text_move_h_pos, int &text_move_v_pos, bool &text_found_flag)
    {
        Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + ",TextMove," + std::to_string(h_Pos_input) + "," + std::to_string(v_pos_input));

        TextVectorIterator text_ptr;

        text_found_flag = false;

        if(!text_vector.empty())
        {
            int x_{static_cast<int>(text_vector.size())};

            for(text_ptr = (text_vector.end() - 1); text_ptr >= text_vector.begin(); text_ptr--)
            {
                x_--;

                // if((h_Pos_input >= text_ptr->h_pos) && (h_Pos_input < (text_ptr->h_pos + abs(text_ptr->Font->Height))) && (VPosInput >= TextPtr->VPos) &&
                // (VPosInput < (TextPtr->VPos + abs(TextPtr->Font->Height))))
                // {
                //     TextItem = x;
                //     TextMoveHPos = TextPtr->HPos;
                //     TextMoveVPos = TextPtr->VPos;
                //     TextFoundFlag = true;
                //     Utilities->CallLogPop(1309);
                //     return;
                // } // if ....

            } // for TextPtr...
        } // if !TextVector...

        Logging::popCallLog(1310);
    }

    bool TextHandler::textFound(int caller, int h_pos_input, int v_pos_input, const std::string& text)
    {
        Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + ",TextFound," + std::to_string(h_pos_input) + "," + std::to_string(v_pos_input));

        TextVectorIterator text_ptr;

        if(text_handler->textVectorSize(1) == 0)
        {
            Logging::popCallLog(1311);
            return false;
        }
        for(text_ptr = (text_handler->text_vector.end() - 1); text_ptr >= text_handler->text_vector.begin(); text_ptr--)
        {
            // if((h_pos_input >= text_ptr->h_pos) && (h_pos_input < (text_ptr->h_pos + abs(text_ptr->font->Height))) && (VPosInput >= TextPtr->VPos) && (VPosInput <
            //                                                                                                                                 (TextPtr->VPos + abs(TextPtr->Font->Height))))
            // {
            //     Text = TextPtr->TextString;
            //     Utilities->CallLogPop(1312);
            //     return(true);
            // }
        }
        Logging::popCallLog(1313);
        return false;
    }
};
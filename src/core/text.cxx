#include "railos/text.hxx"
#include "railos/logging.hxx"
#include "railos/track.hxx"
#include <memory>
#include <stdexcept>
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

    void TextHandler::textVectorPush(int caller, TextItem text)
    {
        Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + ",TextVectorPush," + text.text_string);
        const int h_loc_{text.h_pos / 16};
        const int v_loc_{text.v_pos / 16};

        if(h_loc_ < Track::track->getHLocMin())
        {
            Track::track->setHLocMin(h_loc_);
        }
        if(h_loc_ > Track::track->getHLocMax())
        {
            Track::track->setHLocMax(h_loc_);
        }
        if(v_loc_ < Track::track->getVLocMin())
        {
            Track::track->setVLocMin(v_loc_);
        }
        if(v_loc_ > Track::track->getVLocMax())
        {
            Track::track->setVLocMax(v_loc_);
        }
        text_vector.push_back(text);
        Logging::popCallLog(1330);
    }

    unsigned int TextHandler::textVectorSize(int caller)
    {
        Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + ",TextVectorSize");

        Logging::popCallLog(1430);
        return text_vector.size();
    }

    unsigned int TextHandler::selectTextVectorSize(int caller)
    {
        Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + ",SelectTextVectorSize");

        Logging::popCallLog(1431);
        return select_text_vector.size();
    }

    std::shared_ptr<TextItem> TextHandler::textPtrAt(int caller, int at)
    {
        Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + "," + std::to_string(at) + ",TextPtrAt");
        if(at < 0 || static_cast<unsigned int>(at) >= text_vector.size())
        {
            throw std::runtime_error("At value outside range of TextVector in TextPtrAt");
        }

        Logging::popCallLog(1428);
        return std::shared_ptr<TextItem>(&(text_vector.at(at)));
    }

    std::shared_ptr<TextItem> TextHandler::selectTextPtrAt(int caller, int at)
    {
        Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + "," + std::to_string(at) + ",SelectTextPtrAt");
        if(at < 0 || static_cast<unsigned int>(at) >= text_vector.size())
        {
            throw std::runtime_error("At value outside range of TextVector in TextPtrAt");
        }

        Logging::popCallLog(1429);
        return std::shared_ptr<TextItem>(&(text_vector.at(at)));
    }


    bool TextHandler::findText(int caller, const std::string& name, int &h_pos, int &v_pos)
    {
        // Return true if find name in TextVector, HPos & VPos give location, else return false
        Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + ",FindText," + name);
        for(unsigned int x{0}; x < textVectorSize(12); x++)
        {
            if(text_vector.at(x).text_string == name)
            {
                h_pos = text_vector.at(x).h_pos;
                v_pos = text_vector.at(x).v_pos;
                Logging::popCallLog(1560);
                return true;
            }
        }
        Logging::popCallLog(1559);
        return false;
    }
};
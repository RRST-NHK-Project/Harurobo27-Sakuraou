#pragma once // 多重includeを防ぐためのお守り？

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

enum class Direction
{
    kTx,
    kRx
};

enum class ChannelKind
{
    kMotor,
    kCounter,
    kDigitalIn,
    kRawOut,
    kRawIn
};

struct ChannelDef
{
    std::string name;
    int index;
    ChannelKind kind;
    Direction direction;
};

inline std::string Direction2String(Direction d)
{
    switch (d)
    {
    case Direction::kTx:
        return "Tx";
    case Direction::kRx:
        return "Rx";
    default:
        return "Unknown";
    }
}

inline std::string ChannelKind2String(ChannelKind k)
{
    switch (k)
    {
    case ChannelKind::kMotor:
        return "Motor";
    case ChannelKind::kCounter:
        return "Counter";
    case ChannelKind::kDigitalIn:
        return "DigitalIn";
    case ChannelKind::kRawOut:
        return "RawOut";
    case ChannelKind::kRawIn:
        return "RawIn";
    default:
        return "Unknown";
    }
}

inline Direction expectedDirection(ChannelKind kind)
{
    switch (kind)
    {
    case ChannelKind::kMotor:
    case ChannelKind::kRawOut:
        return Direction::kTx;
    case ChannelKind::kCounter:
    case ChannelKind::kDigitalIn:
    case ChannelKind::kRawIn:
        return Direction::kRx;
    default:
        throw std::invalid_argument("Unknown channel kind");
    }
}
class DeviceProfiles
{ // スロット番号を返すためのクラスの定義
private:
    int node_count_;
    int slots_per_node_;
    std::vector<ChannelDef> channel_defs_;

public:
    int total_slots() const
    {
        return node_count_ * slots_per_node_;
    }

    DeviceProfiles(int node_count, int slots_per_node)
        : node_count_(node_count), slots_per_node_(slots_per_node)
    {
        if (total_slots() > 24 || node_count_ <= 0 || slots_per_node_ <= 0)
        {
            throw std::invalid_argument("無効な1バスごとのノード数、または1ノードごとのスロット数です。ノード数*ノードごとのスロット数は1以上,24以下である必要があります");
        }
    }

    void addChannel(ChannelDef channel_def)// structされたChannelDefという型のchannel_defという変数
    { 
        if(channel_def.name.empty()){
            throw std::invalid_argument("無効な名前です。チャンネルの名前は空文字列にできません");
        }

        if (channel_def.direction != expectedDirection(channel_def.kind))
        {
            throw std::invalid_argument("無効な方向です。チャンネルの種類" + ChannelKind2String(channel_def.kind) + "に対して不適切な方向"  
                                        + Direction2String(channel_def.direction) + "が指定されています");
        }

        if (channel_def.index >= total_slots() ||channel_def.index < 0)
        {
            throw std::invalid_argument("無効なindexです。indexは0以上、total_slots()未満である必要があります");
        }

        for (const ChannelDef &c : channel_defs_ ){//
            if(c.name == channel_def.name){
                throw std::invalid_argument("無効な名前です。すでに同じ名前のチャンネルが存在します:" + channel_def.name);
            }else if (c.direction == channel_def.direction && c.index == channel_def.index){
                throw std::invalid_argument("無効なindexです。すでに" 
                                            + Direction2String(channel_def.direction) + "方向のチャンネル" 
                                            + c.name + "が存在します:" + std::to_string(channel_def.index));
            }

        }




        channel_defs_.push_back(channel_def);
    }

    int Return_SlotIndex(int node_index, int local_slot_id)
    {
        if (node_index >= node_count_ || local_slot_id >= slots_per_node_ || node_index < 0 || local_slot_id < 0 || node_index * slots_per_node_ + local_slot_id >= 24) //
        {
            std::cout << "Invalid slot index" << std::endl;
            return -1;
        }
        return (node_index * slots_per_node_) + local_slot_id;
    }
};

/*
DeviceProfiles: ros2can の24スロットフレームを「名前付きチャンネル」として扱うための部品
Copyright (c) 2026 RRST-NHK-Project
ライセンス: Apache-2.0 (package.xml / LICENSE 参照)

[このファイルの役割]
ros2can (CANホスト) とやり取りする serial_tx_[DEVICE_ID] / serial_rx_[DEVICE_ID] は
int16 x 24 スロットの配列で、どのスロットが何を意味するかは配列自体には書かれていない。
このファイルは「24スロットをどう区切るか」と「各スロットが何か (チャンネル)」を定義し、
間違った設定を登録した時点で弾くための部品。
hardware_interface ノード (複数ノードの指令を1枚のフレームに合成する。未作成) の
土台になる予定。
ROSには依存しない (rclcpp を include しない) ので、単体でテストできる。

[スロットの区切り方]
  24スロットを CANバス上の node_count 台へ slots_per_node ずつ分配する
  (ros2can の device_profiles.py / ファームの config.hpp の CAN_NODE_COUNT,
   CAN_SLOTS_PER_NODE と同じ考え方)。
  グローバル index = node_index * slots_per_node + local
  node_index は 0始まり。CAN_ID とは node_index = (CAN_ID % 100) - 1 の関係
  (例: CAN_ID 101 = node0。桜凰では足回り基板 (ホスト、設計中) を node0 にする予定)。

[チャンネル]
  チャンネル = 意味を付けたスロット1つ (名前・index・種類・向き)。
  同じ index でも TX (PC -> マイコン) と RX (マイコン -> PC) は別のスロット。
  種類と向きの組み合わせは決まっている:
    TX: kMotor, kRawOut
    RX: kCounter, kDigitalIn, kRawIn

[登録時に弾くもの (std::invalid_argument)]
  - 構成: node_count / slots_per_node が 0以下、または合計が24を超える
  - チャンネル: 名前が空 / 種類と向きの食い違い / index が範囲外 /
               名前の重複 / 同じ向きで同じ index の重複
  - expectedDirection(): 未知の ChannelKind (通常は起きない。enum に種類を足して
                         ここの switch を直し忘れたときの保険)
  設定ミスは「動いている最中」ではなく「起動した瞬間」に見つけたいので、
  ここでは戻り値ではなく例外で止める (esc_ctrl / cubemars_ctrl も、起動時のパラメータ
  チェックで同じようにしている)。

[例外ではなく -1 を返すもの]
  Return_SlotIndex(node_index, local): 範囲外なら -1 を返す
  (動作中にも呼ぶ可能性がある計算用の関数なので、ノードを落とさないようにしている)。

[使い方]
  DeviceProfiles foot(1, 5);  // 足回り基板1台、1ノードあたり5スロット
  foot.addChannel({"drive_md1", 0, ChannelKind::kMotor,   Direction::kTx});
  foot.addChannel({"drive_enc1", 0, ChannelKind::kCounter, Direction::kRx});
*/
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

import numpy as np

def micro_pre(nodes_per_room, f_p_per_room):
    assert(len(nodes_per_room) == len(f_p_per_room))

    t_p_per_room = []
    for t, fp in zip(nodes_per_room, f_p_per_room):
        t_p_per_room.append(t - fp)

    pre_micro_num = 0;
    pre_micro_den = 0;
    for tp, fp in zip(t_p_per_room, f_p_per_room):
        pre_micro_num += tp
        pre_micro_den += (tp+fp)

    pre_micro = pre_micro_num / pre_micro_den
    return pre_micro

def macro_pre(nodes_per_room, f_p_per_room):
    assert(len(nodes_per_room) == len(f_p_per_room))

    t_p_per_room = []
    for t, fp in zip(nodes_per_room, f_p_per_room):
        t_p_per_room.append(t - fp)

    all_pres = 0
    for tp, fp in zip(t_p_per_room, f_p_per_room):
        a= tp
        b= (tp+fp)
        all_pres += a/b

    return all_pres / len(t_p_per_room)


if __name__ == "__main__":
    print("Calculating recalls / precision")

    nodes_per_room = [100, 
                      100,
                      100,
                      100,
                      100,
                      100,
                      100,
                      100,
                      100,
                      100,
                      100,
                      100,
                      100,
                      100,
                      100]

    f_p_per_room = [2,
                    2,
                    2,
                    2,
                    2,
                    2,
                    2,
                    2,
                    2,
                    2,
                    2,
                    2,
                    2,
                    2,
                    2]

    val = macro_pre(nodes_per_room, f_p_per_room)
    print(val)


                      

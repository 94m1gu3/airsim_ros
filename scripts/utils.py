
GAME_OBJECTS = {
    # "Floor": {"values": 1, 'regex':True}, 
    # "Wall": {"values": 161, 'regex':True},
    "Obj_Cube": {"values": 161, 'regex':True},
    "Obj_Cylinder": {"values": 11, 'regex':True},
    }


def reset_objects_id(client, default_value=0):
    client.simSetSegmentationObjectID("[{0}w]*".format("\\"), default_value, True)


def set_objects_id(client, id_dict=GAME_OBJECTS):
    for name in id_dict:
        # para usar expresiones regulares el ultimo parametro debe ser verdadero (por defecto, es false)
        value, is_regex = id_dict[name]['values'], id_dict[name]['regex']
        client.simSetSegmentationObjectID(name, value, is_regex)
        # print("Done: %r" % (found))
from lanelet2_traffic_light.frontend_editor.arrow_toggle import is_vehicle_tl_label


def test_is_vehicle_tl_label_true_for_tlv():
    assert is_vehicle_tl_label("TLV_12345") is True


def test_is_vehicle_tl_label_false_for_pedestrian_and_others():
    assert is_vehicle_tl_label("TLP_999") is False
    assert is_vehicle_tl_label("TL_1") is False
    assert is_vehicle_tl_label("Traffic_Lights_5") is False

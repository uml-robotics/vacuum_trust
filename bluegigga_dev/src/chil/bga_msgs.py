import jsonschema
import ujson as json
import bga_schema

ACK_TYPE = "ack"
REPLY_TYPE = "reply"
UPDATE_TYPE = "update"

# TODO: based on an environment variable run the validates

#def img_msg(file_name):
#    with open("res/" + file_name, 'rb') as img_text:
#        encoded_img = base64.b64encode(img_text.read())
#    return json.dumps({
#        "json_type": IMG_JT,
#        "file_name": file_name,
#        "img": encoded_img})

def ack_msg():
    ack_msg = {"msgtype": "ack"}
    bga_schema.validate_object(ack_msg, bga_schema.ack_schema)
    return json.dumps(ack_msg)

def update_msg(name, model, state, progression=None):
    update_msg = {
            "msgtype": "status",
            "name": name,
            "model": model,
            "state": state,
            }
    if progression != None:
        update_msg['progression'] = progression
    bga_schema.validate_object(update_msg, bga_schema.update_schema)
    return json.dumps(update_msg)

def prog_element(msgid, content, responses, popup=None, selection=None, res=None):
    prog_element = {
            "msgid": msgid,
            "content": content,
            "responses": responses,
            "popup": (popup != None and popup != False),
            }
    if selection != None:
        prog_element['selection'] = selection
    if res != None:
        prog_element['res'] = res
    return prog_element

def dict_element(key, value):
    return {
            "id": key,
            "value": value
            }

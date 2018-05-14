import jsonschema

def validate_object(msg, schema):
    """ Validate a python object as matching a schema
    Example:
    validate_object([{"id": "myid", "value": "myvalue"}], dict_schema)
    validate_object({"name": "BB-8", "model": "droid", "state": "ok"}, update_schema)
    """
    try:
        jsonschema.validate(msg, schema)
    except Exception as e:
        print e
        return False
    return True

def validate_schema(schema):
    """ Validate that the schemas above are valid schemas """
    try:
        jsonschema.Draft4Validator.check_schema(schema)
    except Exception as e:
        print e
        return False
    return True

dict_schema = {
    "type": "array",
    "minItems": 1,
    "items": {
        "type": "object",
        "properties": {
            "id": {"type": "string"},
            "value": {"type": "string"}
            },
        "required": ["id","value"],
        "additionalProperties": False
        },
    "additionalProperties": False
    }

status_schema = {
    "type": "object",
    "properties":{
        "msgid": {"type": "string"},
        "content": {"type": "string"},
        "res": dict_schema,
        "responses": dict_schema,
        "selection": {"type": "string"}, # the id of a response
        "popup": {"type": "boolean"} # whether or not to make this a push-interaction
        },
    "required": ["msgid", "content", "popup"],
    "additionalProperties": False
    }

# Update Type Message
# {"msgtype": "status",
#  "name": "BB-8",
#  "model": "droid",
#  "state": "ok",
#  "progression": [{
#           "msgid": "opendoor",
#           "content": "Could you please open the door for me? @doorpic",
#           "res": [{"id": "@doorpic", "value": "/app/res/door.jpg"}],
#           "responses": [{"id": "yes", "value": "Sure"},
#                         {"id": "no", "value": "No thanks"}]
#           }]
#   }
update_schema = {
    "type": "object",
    "properties": {
        "msgtype": {"enum": ["status"]},
        "name": {"type": "string"},
        "model": {"type": "string"},
        "state": {"enum": ["ok", "safe", "help", "dangerous", "off"]},
        "progression": {
            "type": "array",
            "minItems": 1,
            "items": status_schema
            }
        },
    "required": ["msgtype", "name", "model", "state"],
    "additionalProperties": False
    }

# Reply Type Message
# {"msgtype": "reply",
#  "responses": [{"id": "opendoor", "value": "yes"}]
# }
reply_schema = {
    "type": "object",
    "properties": {
        "msgtype": {"enum": ["reply"]},
        "responses": dict_schema,
        },
    "required": ["msgtype", "responses"],
    "additionalProperties": False
    }

# Ack Type Message
# {"msgtype": "ack"}
ack_schema = {
    "type": "object",
    "properties": {
        "msgtype": {"enum": ["ack"]}
        },
    "required": ["msgtype"],
    "additionalProperties": False
    }

# CHIL Top Level Schema.
chil_schema = {
    "type": "object",
    "oneOf": [update_schema, reply_schema, ack_schema]
    }

import asyncio
import json
import sys

try:
    import websockets
except ImportError:
    print("Websockets package not found. Make sure it's installed.") 

# For local streaming, the websockets are hosted without ssl - ws://
HOST = '192.168.1.41:5005'
URI = f'ws://{HOST}/api/v1/stream'

# For reverse-proxied streaming, the remote will likely host with ssl - wss://
# URI = 'wss://your-uri-here.trycloudflare.com/api/v1/stream'

async def run(context):
    # Note: the selected defaults change from time to time.
    
    
    request = {
        'prompt': context,
        'max_new_tokens': 120,

        # Generation params. If 'preset' is set to different than 'None', the values
        # in presets/preset-name.yaml are used instead of the individual numbers.
        'preset': 'None',  
        'do_sample': True,
        'temperature': 0.7,
        'top_p': 0.1,
        'typical_p': 1,
        'epsilon_cutoff': 0,  # In units of 1e-4
        'eta_cutoff': 0,  # In units of 1e-4
        'tfs': 1,
        'top_a': 0,
        'repetition_penalty': 1.18,
        'repetition_penalty_range': 0,
        'top_k': 40,
        'min_length': 0,
        'no_repeat_ngram_size': 0,
        'num_beams': 1,
        'penalty_alpha': 0,
        'length_penalty': 1,
        'early_stopping': False,
        'mirostat_mode': 0,
        'mirostat_tau': 5,
        'mirostat_eta': 0.1,

        'seed': -1,
        'add_bos_token': True,
        'truncation_length': 2048,
        'ban_eos_token': False,
        'skip_special_tokens': True,
        'stopping_strings': ["\n### Human:", "\n### Orange:", "\n### Human Commands:", "\n### Robot Commands:"]
    }

    async with websockets.connect(URI, ping_interval=None) as websocket:
        await websocket.send(json.dumps(request))

        #yield context # Remove this if you just want to see the reply

        while True:
            incoming_data = await websocket.recv()
            incoming_data = json.loads(incoming_data)

            s = incoming_data['event']
            if s == 'text_stream':
                yield incoming_data['text']
            elif s == 'stream_end':
                return

async def get_response_sentence(prompt):
    sentence = ""
    while True:
        async for response in run(prompt):
            sentence += response
            if (response == '.' or response == '!' or response == '\n' or response == ";") and len(sentence) > 25:
                temp = sentence
                sentence = ""
                yield temp
        return

async def print_response_sentences(prompt):
    async for sentence in get_response_sentence(prompt):
        print(sentence, end='')
        sys.stdout.flush()

async def print_response_stream(prompt):
    async for response in run(prompt):
        print(response, end='|')
        sys.stdout.flush() # If we don't flush, we won't see tokens in realtime.

if __name__ == '__main__':
    prompt = "A chat between a human and a robot assistant named orange\n### Human:list top places to retire in the u.s.?\n### Orange:"
    asyncio.run(print_response_sentences(prompt))

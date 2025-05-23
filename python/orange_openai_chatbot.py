from openai_chatbot import OpenAiChatbot

class OrangeOpenAiChatbot(OpenAiChatbot):
    def __init__(self):
        prompt_name ="orange_prompt"
        init_prompt = ""
        with open(f'prompts/{prompt_name}.txt', 'r') as f:
            init_prompt = f.read()
        
        with open(f'prompts/{prompt_name}_messages.txt', 'r') as f:
            prompt_messages = f.read()
        
        intro_line = "In chat mode I can answer questions about myself. Say goodbye to end the chat. What can I answer for you today?"
        super().__init__("gpt-4o", init_prompt, intro_line, "Human", "Robot", prompt_messages)
        

import base64
if __name__ == "__main__":
    chat_bot = OrangeOpenAiChatbot()
    inp = ""
    
    #load jpeg image and convert to base64
    # with open("C:/Users/LattePanda/Pictures/Camera Roll/WIN_20210118_22_22_48_Pro.jpg", "rb") as image_file:
    #     image = image_file.read()

    print(chat_bot.intro_line)
    while inp != "bye":
        print("> ",end='')
        inp = input()
        if inp == ".log":
            print(chat_bot.get_log())
            continue
        answer = chat_bot.get_response(inp)
        chat_bot.add_to_chat_log(answer)
        print(answer)
        
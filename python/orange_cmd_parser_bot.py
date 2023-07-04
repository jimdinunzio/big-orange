from textgen_chatbot import TextGenChatbot
import asyncio

class OrangeTextGenChatbot(TextGenChatbot):
    def __init__(self):
        prompt_name ="orange_cmd_parser_prompt"
        init_prompt = ""
        with open(f'prompts/{prompt_name}.txt', 'r') as f:
            init_prompt = f.read()
        
        with open(f'prompts/{prompt_name}_messages.txt', 'r') as f:
            prompt_messages = f.read()
        
        intro_line = "Give a human command and it will be translated to robot commands."
        super().__init__("", init_prompt, intro_line, "### Human Commands", "### Robot Commands", prompt_messages) 
        
if __name__ == "__main__":


    chat_bot = OrangeTextGenChatbot()
    inp = ""
    print(chat_bot.intro_line)
    while inp != "bye":
        async def print_response(inp):
            response = ""
            async for sent in chat_bot.get_response_stream(inp):
                if len(sent) > 0:
                    print(sent)
                    response += sent
            return response

        print("> ",end='')
        inp = input()
        if inp == ".log":
            print(chat_bot.get_log())
            continue
        elif inp == ".reset":
            chat_bot.init_chat_log()
            continue

        print("response = ", asyncio.run(print_response(inp)))
       
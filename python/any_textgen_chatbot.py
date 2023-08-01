from textgen_chatbot_chat import TextGenChatbot
import asyncio
import argparse

class AnyTextGenChatbot(TextGenChatbot):
    def __init__(self, prompt_name):
        init_prompt = ""
        with open(f'prompts/{prompt_name}.txt', 'r') as f:
            init_prompt = f.read()
        
        try:
            with open(f'prompts/{prompt_name}_messages.txt', 'r') as f:
                prompt_messages = f.read()
        except:
            None
        
        intro_line = "In chat mode I can converse freely. Say goodbye to end the chat. What can I answer for you today?"
        super().__init__("", init_prompt, intro_line, "User", "RoBud")
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("prompt_name")
    args = parser.parse_args()    
    chat_bot = AnyTextGenChatbot(args.prompt_name + "_prompt")
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
       
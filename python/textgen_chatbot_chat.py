import textgen_client_chat as tc
import asyncio
import sys

class TextGenChatbot:
    def __init__(self, mname, init_prompt, intro_line, name1_label, name2_label, prompt_messages=""):
        self.mname = mname
        self.init_prompt = init_prompt + prompt_messages
        self.intro_line = intro_line
        self.chat_log = ''
        self.reply_ids = None
        self.name1_label = name1_label
        self.name2_label = name2_label
        self.history = {'internal': [], 'visible': []}

    def get_intro_line(self):
        return self.intro_line
    
    async def get_response_stream(self, user_input):
        print(f'submitting:\n{user_input}\n-------------------------------\n')
        resp = ""

        while True:
            async for sentence in tc.get_response_sentence(user_input, self.history, self.name1_label):
                resp += sentence
                yield sentence
        
            #resp = full_resp.replace(f"{self.init_prompt}{self.chat_log}", "")
            #print(f"\n------------------------------\n{resp}\n-----------------------------\n")
            self.add_to_chat_log(input, resp)
            return
    
    async def print_response(self, input):
        async for sent in self.get_response_stream(input):
            print(sent, end='<end>')
            sys.stdout.flush()

    def add_to_chat_log(self, name1_utt, name2_utt):
        exchange = []
        exchange.append(name1_utt)
        exchange.append(name2_utt)
        self.history['internal'].append(exchange)
        self.history['visible'].append(exchange)
        self.chat_log += f'\n{self.name1_label}:{name1_utt}\n{self.name2_label}:{name2_utt}'

    def init_chat_log(self):
        self.chat_log = ''

    def get_log(self):
        return self.chat_log
    
if __name__ == '__main__':
    prompt = "list top places retire in the u.s.?"
    a = TextGenChatbot("vicuna", "A chat between a human and an assistant","", "### Human", "### Assistant")
    asyncio.run(a.print_response(prompt))
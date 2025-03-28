import os
import openai
import base64

openai.api_key = os.getenv("OPENAI_API_KEY")

#roles: system, user, or assistant
# user is the role that gives instructions or asks questions
# system is the role that responds. content contains description of that role (e.g. friendly assistant)
# assistant is the role that 

# messages=[
#         {
#             "role": "user",
#             "content": [
#                 { "type": "text", "text": "what's in this image?" },
#                 {
#                     "type": "image_url",
#                     "image_url": {
#                         "url": f"data:image/jpeg;base64,{base64_image}",
#                     },
#                 },
#             ],
#         }
#     ],

class OpenAiChatbot:
    def __init__(self, mname, init_prompt, intro_line, name1_label, name2_label, prompt_messages):
        self.mname = mname
        self.init_prompt = init_prompt
        self.intro_line = intro_line
        self.name1_label = name1_label
        self.name2_label = name2_label
        self.prompt_messages = prompt_messages
        self.log_start = 0
        self.init_chat_log()

    def get_intro_line(self):
        return self.intro_line
    
    def get_response(self, input, image=None):
        if image is None:
            self.messages.append({"role": "user", "content": input})
        else:
            base64_image = base64.b64encode(image).decode('utf-8')
            messsage = {
                "role": "user",
                "content": [
                    { "type": "text", "text": input },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{base64_image}",
                        },
                    },
                ],
            }
            self.messages.append(messsage)

        completion = openai.ChatCompletion.create(
            model=self.mname,
            #top_p=1.0,                # range 0 to 1.0, default 1.0, alternative to temp, 0.1 means only top 10% probability mass are considered
            #temperature=0.9,          # range: 0.0 to 2.0, default 0.8  Higher values like 0.8 make output more random, lower more deterministic
            max_tokens=50,            # maximum number of tokens to generate in the completion (gpt3: 2048, GPT3.5: 4096)
            #frequency_penalty=0.9,    # range: -2.0 to 2.0, default 0
            #n=1,                      # how many chat completion choices to generate for each input message
            messages=self.messages
            )

        res = completion.choices[0].message.content
        if "." in res:
            res = res.strip().rpartition('.')[0]            # strip white space and take up to last period.
            if len(res):
                res += '.'
        return res
    
    def add_to_chat_log(self, response):
        self.messages.append({"role": "assistant", "content": response})

    def init_chat_log(self):
        self.messages = [{"role": "system", "content" : self.init_prompt}]
        prompt_lines = self.prompt_messages.split('\n')
        line_count = len(prompt_lines)
        
        #find closest even number less than line count
        line_count = line_count^1 if line_count&1 else line_count
        self.log_start_message_index = line_count + 1
        for index in range(0, line_count, 2):
            self.messages.append({"role": "user", "content": prompt_lines[index].split(":")[1].strip()})
            self.messages.append({"role": "assistant", "content": prompt_lines[index+1].split(":")[1].strip()})

    def get_log(self):
        log = ""
        end = len(self.messages)
        for index in range(self.log_start_message_index, end):
            msg = self.messages[index]
            if msg['role'] == 'user':
                log += f"{self.name1_label}: {msg['content']}\n"
            elif msg['role'] == 'assistant':              
                log += f"{self.name2_label}: {msg['content']}\n"
        return log
            

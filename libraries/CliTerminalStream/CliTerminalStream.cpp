#include "CliTerminalStream.h"

Command::Command(char *command_string, void (*callback)(String))
{
    command_str = command_string;
    _callback = callback;
}
void Command::Execute(String str)
{
    _callback(str);
}
Command::~Command()
{
}


void Cli_Terminal::cli_cal(Stream &streamInput)
{
    stream = &streamInput;
	if (stream->available())
    {
        int length = stream->available();
        if (length > CONSOLE_MAX_LENGTH)
        {
            stream->println("command is too long !! process terminated...");
            return;
        }
        String command = stream->readStringUntil((char)10);
        //Serial.println(command);
        String processed[2];
        *processed = *TextProcessor(command, processed);
        int SettedCommandCount = sizeof(commands) / sizeof(commands[0]);
        for (int itr = 0; itr < SettedCommandCount; itr++)
        {
#ifdef DEBUG
            stream->println(itr);
            stream->println(commands[itr].command_str);
            stream->println(processed[0] == commands[itr].command_str ? "true" : "false");
#endif
            if (processed[0] == commands[itr].command_str)
            {
                commands[itr].Execute(processed[1]);
                break;
            }
            if (itr == SettedCommandCount - 1)
                stream->println(processed[0] + ": command not found");
        }
        stream->flush();
        stream->print(">");
    }
}

String *Cli_Terminal::TextProcessor(String text, String *result)
{
    int space = text.indexOf(" ");
    result[0] = text; //for No Attribute Command
    if (space > 0)
    {
        result[0] = text.substring(0, space);
        result[1] = text.substring(++space, text.length());
    }

#ifdef DEBUG
    stream->println(result[0]);
    stream->println(result[1]);
#endif
    return result;
}
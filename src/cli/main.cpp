#include <QCoreApplication>
#include <QStringList>
#include <QTextStream>

#include "cli/commands.h"
#include "cli/session.h"
#include "cli/term.h"
#include "cli/tui.h"

// ServoBench, terminal edition. With a command it does one thing and exits;
// with nothing to do it starts the full-screen tool, which is the window's
// three tabs driven from the keyboard.
int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("servobench-cli");
    QCoreApplication::setApplicationVersion("1.0");

    QStringList args;
    for(int i = 1; i < argc; i++)
        args << QString::fromLocal8Bit(argv[i]);

    const cli::CommandLine line = cli::parseCommandLine(args);

    if(!line.error.isEmpty())
    {
        QTextStream(stderr) << "servobench: " << line.error << "\n";
        return 2;
    }

    if(line.flags.contains("version"))
    {
        QTextStream(stdout) << "servobench-cli 1.0 (Qt " << qVersion() << ")\n";
        return 0;
    }

    if(line.flags.contains("help"))
    {
        if(line.command.isEmpty())
            cli::printUsage();
        else
            cli::printCommandHelp(line.command);
        return 0;
    }

    if(line.flags.contains("plain"))
        term::colors = false;
    if(line.flags.contains("ascii"))
        term::unicode = false;

    if(line.command.isEmpty() || line.command.toLower() == "tui")
    {
        cli::ConnectionOptions options;
        QString error;
        if(!cli::connectionFrom(line, &options, &error))
        {
            QTextStream(stderr) << "servobench: " << error << "\n";
            return 2;
        }

        // With no --port, start on the first USB serial adapter, which is what
        // the window's dropdown lands on too.
        if(options.port.isEmpty())
        {
            const QVector<cli::PortInfo> ports = cli::availablePorts();
            if(!ports.isEmpty())
                options.port = ports.first().name;
        }

        cli::Tui tui(options);
        return tui.run();
    }

    return cli::runCommand(line);
}

#ifndef CLI_COMMANDS_H
#define CLI_COMMANDS_H

#include <QMap>
#include <QSet>
#include <QString>
#include <QStringList>

#include "cli/session.h"

namespace cli
{

// A parsed argument vector: the command word, its positional arguments, and
// the options and flags that came with them in any order.
struct CommandLine
{
    QString command;
    QStringList positional;
    QMap<QString, QString> options;
    QSet<QString> flags;
    QString error;

    bool has(const QString &name) const { return flags.contains(name) || options.contains(name); }
    QString value(const QString &name, const QString &fallback = QString()) const;
    int intValue(const QString &name, int fallback, bool *ok = nullptr) const;
    double doubleValue(const QString &name, double fallback, bool *ok = nullptr) const;
};

CommandLine parseCommandLine(const QStringList &args);

// Connection settings from --port/--baud/--parity/--timeout.
bool connectionFrom(const CommandLine &line, ConnectionOptions *out, QString *error);

// Runs one non-interactive command. Returns a process exit code: 0 for
// success, 1 for a failure on the bus, 2 for a usage mistake.
int runCommand(const CommandLine &line);

void printUsage();
void printCommandHelp(const QString &command);

// True once Ctrl-C has been pressed, so the long-running commands can stop at
// a sample boundary and still flush what they have.
bool interrupted();
void installInterruptHandler();

}

#endif

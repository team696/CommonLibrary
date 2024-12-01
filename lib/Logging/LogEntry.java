// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team696.lib.Logging;

import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.ProtobufLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.protobuf.Protobuf;
import edu.wpi.first.util.protobuf.ProtobufSerializable;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import us.hebi.quickbuf.ProtoMessage;
@SuppressWarnings("unchecked")
/** Add your docs here. */
public class LogEntry<T> {
    public final T value;
    public final String name;
    public final Long time;
    public LogEntry(String name, T value) {
        this.value = value;
        this.name = name;
        this.time = System.currentTimeMillis();

        if (BackupLogger.log == null) return;

        if (publishers.containsKey(name)) return;

        if (value instanceof Integer || value instanceof Long || value instanceof Short) {
            publishers.put(name, new GenericPublisher<>(NetworkTableInstance.getDefault().getIntegerTopic(name).publish()));

            logEntries.put(name, new GenericDataLog<DataLogEntry>(new IntegerLogEntry(BackupLogger.log, name)));
            return;
        }

        if (value instanceof int[] || value instanceof long[] || value instanceof short[]) {
            publishers.put(name, new GenericPublisher<>(NetworkTableInstance.getDefault().getIntegerArrayTopic(name).publish()));

            logEntries.put(name, new GenericDataLog<DataLogEntry>(new IntegerArrayLogEntry(BackupLogger.log, name)));
            return;
        }

        if (value instanceof Double || value instanceof Float) {
            publishers.put(name, new GenericPublisher<>(NetworkTableInstance.getDefault().getDoubleTopic(name).publish()));

            logEntries.put(name, new GenericDataLog<DataLogEntry>(new DoubleLogEntry(BackupLogger.log, name)));
            return;
        }

        if (value instanceof double[] || value instanceof float[]) {
            publishers.put(name, new GenericPublisher<>(NetworkTableInstance.getDefault().getDoubleArrayTopic(name).publish()));

            logEntries.put(name, new GenericDataLog<DataLogEntry>(new DoubleArrayLogEntry(BackupLogger.log, name)));
            return;
        }

        if (value instanceof Boolean) {
            publishers.put(name, new GenericPublisher<>(NetworkTableInstance.getDefault().getBooleanTopic(name).publish()));

            logEntries.put(name, new GenericDataLog<DataLogEntry>(new BooleanLogEntry(BackupLogger.log, name)));
            return;
        } 

        if (value instanceof boolean[]) {
            publishers.put(name, new GenericPublisher<>(NetworkTableInstance.getDefault().getBooleanArrayTopic(name).publish()));

            logEntries.put(name, new GenericDataLog<DataLogEntry>(new BooleanArrayLogEntry(BackupLogger.log, name)));
            return;
        } 


        if(value instanceof StructSerializable) {
            try {
                Struct<?> structImplementation = (Struct<?>)value.getClass().getDeclaredField("struct").get(null);
                publishers.put(name, new GenericPublisher<>(NetworkTableInstance.getDefault().getStructTopic(name, structImplementation).publish()));
            
                logEntries.put(name, new GenericDataLog<DataLogEntry>(StructLogEntry.create(BackupLogger.log, name, structImplementation)));

            } catch ( Exception e ){
                PLog.info("Logger", "Malformed Struct detected: " + name);
            }
            return;
        }


        if(value instanceof StructSerializable[]) {
            try {
                Struct<?> structImplementation = (Struct<?>)value.getClass().getComponentType().getDeclaredField("struct").get(null);
                publishers.put(name, new GenericPublisher<>(NetworkTableInstance.getDefault().getStructArrayTopic(name, structImplementation).publish()));
            
                logEntries.put(name, new GenericDataLog<DataLogEntry>(StructArrayLogEntry.create(BackupLogger.log, name, structImplementation)));

            } catch ( Exception e ){
                PLog.info("Logger", "Malformed Struct detected: " + name);
            }
            return;
        }

        if (value instanceof ProtobufSerializable) {
            try {
                Protobuf<?, ProtoMessage<?>> protobufImplementation = (Protobuf<?, ProtoMessage<?>>)value.getClass().getDeclaredField("proto").get(null);
                publishers.put(name, new GenericPublisher<>(NetworkTableInstance.getDefault().getProtobufTopic(name, protobufImplementation).publish()));
           
                logEntries.put(name, new GenericDataLog<DataLogEntry>(ProtobufLogEntry.create(BackupLogger.log, name, protobufImplementation)));
            } catch ( Exception e ){
                PLog.info("Logger", "Malformed Protobuf detected: " + name);
            }
            return;
        }

        if (value instanceof String) {
            publishers.put(name, new GenericPublisher<>(NetworkTableInstance.getDefault().getStringTopic(name).publish()));

            logEntries.put(name, new GenericDataLog<DataLogEntry>(new StringLogEntry(BackupLogger.log, name)));

            return;
        }

        PLog.info("Logger", "failed to categorize logEntry: " + name);

    }   
    

    /**
     *         private final StructArrayPublisher<SwerveModuleState> swerveModuleDesiredStatePublisher = NetworkTableInstance.getDefault()
.getStructArrayTopic("696/Swerve/DesiredStates", SwerveModuleState.struct).publish();

     */

     public static class GenericPublisher<T extends Publisher> {
        public T publisher;
        public GenericPublisher(T publisher) {
            this.publisher = publisher;
        }
     }

     public static class GenericDataLog<T extends DataLogEntry> {
        public T dataLog;
        public GenericDataLog(T log) {
            this.dataLog = log;
        } 
     }
    public static final ConcurrentHashMap<String, GenericPublisher<?>> publishers = new ConcurrentHashMap<>();
    public static final ConcurrentHashMap<String, GenericDataLog<?>> logEntries = new ConcurrentHashMap<>();
}

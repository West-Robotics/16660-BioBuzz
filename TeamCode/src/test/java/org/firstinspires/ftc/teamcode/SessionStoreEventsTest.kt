package org.firstinspires.ftc.teamcode

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Test
import java.io.File
import java.sql.DriverManager

/**
 * JVM tests proving [SessionStore.events] scopes [event_field] to the
 * requested session, so fields never leak across sessions that share one
 * database file.
 */
class SessionStoreEventsTest {

    /** Opens a fresh session database and returns its store plus connection. */
    private fun openStore(path: String): Pair<SessionStore, java.sql.Connection> {
        val file = File(path)
        file.parentFile?.mkdirs()
        val conn = DriverManager.getConnection("jdbc:sqlite:${file.absolutePath}")
        return SessionStore(JdbcSql(conn)).createSchema() to conn
    }

    @Test
    fun `event fields do not leak across two sessions in one database`() {
        val (store, conn) = openStore(File.createTempFile("two-sessions", ".db").absolutePath)
        try {
            val s1 = store.beginSession("session-a", "auto")
            val s2 = store.beginSession("session-b", "teleop")

            val now = System.nanoTime()
            store.addEvent(s1, 0, SessionEvent(now, now, "phase", listOf("name" to "1-collect", "cycle" to 1)))
            store.addEvent(s1, 1, SessionEvent(now, now, "pickup", listOf("label" to "Ball", "how" to "intake")))
            store.addEvent(s2, 0, SessionEvent(now, now, "phase", listOf("name" to "1-collect", "cycle" to 7)))
            store.addEvent(s2, 1, SessionEvent(now, now, "fire_done", listOf("fired" to 3, "attempted" to 3)))

            // Index each session's events by type -> field map.
            val e1 = store.events(s1).associateBy({ it.first["type"] as? String }, { it.second })
            val e2 = store.events(s2).associateBy({ it.first["type"] as? String }, { it.second })

            assertEquals(2, e1.size)
            assertEquals(2, e2.size)

            // Session 1's events carry only session 1's fields.
            assertEquals("1-collect", e1["phase"]!!["name"])
            assertEquals(1, (e1["phase"]!!["cycle"] as Number).toInt())
            assertEquals("Ball", e1["pickup"]!!["label"])
            assertEquals("intake", e1["pickup"]!!["how"])

            // Session 2's events carry only session 2's fields.
            assertEquals("1-collect", e2["phase"]!!["name"])
            assertEquals(7, (e2["phase"]!!["cycle"] as Number).toInt())
            assertEquals(3, (e2["fire_done"]!!["fired"] as Number).toInt())
            assertEquals(3, (e2["fire_done"]!!["attempted"] as Number).toInt())

            // Neither session sees the other session's fields.
            assertFalse("session-2 field leaked into session 1", e1.containsKey("fire_done"))
            assertEquals(1, (e1["phase"]!!["cycle"] as Number).toInt()) // not 7
            assertFalse("session-1 field leaked into session 2", e2.containsKey("pickup"))
            assertEquals(7, (e2["phase"]!!["cycle"] as Number).toInt()) // not 1
        } finally {
            conn.close()
        }
    }
}
